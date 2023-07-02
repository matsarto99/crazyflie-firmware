/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * something about the code
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "app.h"
#include "app_channel.h"

#include "commander.h"
#include "system.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "OBSTACLE_AVOIDANCE"
#define HOVERING_HEIGHT 0.4f
#define THRESH 0.1f
#define IMG_W 320
#define IMG_H 240
#define MARGIN 5
#define N_WAYPOINTS 11

// Custom data types

typedef struct __attribute__((packed)) {
  float info;
  float obj_id;
  float xmin;
  float ymin;
  float xmax;
  float ymax;
} PacketRX;

typedef struct __attribute__((packed)) {
  char message_type;
  char info;
} infoPacketTX;

typedef struct __attribute__((packed)) {
  char message_type;
  float time_target_reached;
  float time_planner_avg;
} dataPacketTX;

typedef enum {
  GROUND,
  HOVERING,
  READY,
  STOPPING
} State;

typedef struct {
  float x;
  float y;
  float z;
  float yaw;
} Pose;


// Global variables
static State state = GROUND;
static float landing_height = 0.15f;

// positive yaw -> turn left
// positive yaw rate -> turn right

// SQUARE
// Pose target_points[N_WAYPOINTS] = { // x y z yaw
//                                  {0.0, 0.0, HOVERING_HEIGHT, 0},
//                                  {0.5, 0.0, HOVERING_HEIGHT, 0},
//                                  {0.5, 0.0, HOVERING_HEIGHT, 90},
//                                  {0.5, 0.5, HOVERING_HEIGHT, 90},
//                                  {0.5, 0.5, HOVERING_HEIGHT, 180},
//                                  {0.0, 0.5, HOVERING_HEIGHT, 180},
//                                  {0.0, 0.5, HOVERING_HEIGHT, 270},
//                                  {0.0, 0.0, HOVERING_HEIGHT, 270},
//                                  {0.0, 0.0, HOVERING_HEIGHT, 360},
//                                  {0.0, 0.0, HOVERING_HEIGHT, 0} 
//                                 };

// LEFT, RIGHT, FORTH, LEFT, RIGHT, BACK
Pose target_points[N_WAYPOINTS] = { // x y z yaw
                                 {0.0, 0.0, HOVERING_HEIGHT, 0},
                                 {0.0, 0.5, HOVERING_HEIGHT, 0},
                                 {0.0, -0.5, HOVERING_HEIGHT, 0},
                                 {0.0, 0.0, HOVERING_HEIGHT, 0},
                                 {0.5, 0.0, HOVERING_HEIGHT, 0},
                                 {1.0, 0.0, HOVERING_HEIGHT, 0},
                                 {2.0, 0.0, HOVERING_HEIGHT, 0},
                                 {2.0, 0.5, HOVERING_HEIGHT, 0},
                                 {2.0, -0.5, HOVERING_HEIGHT, 0},
                                 {2.0, 0.0, HOVERING_HEIGHT, 0},
                                 {0.0, 0.0, HOVERING_HEIGHT, 0}
                                };

int curr_target = 0;
int target_counter = 0;
clock_t t_target_begin;
double t_target;

PacketRX rxPacket;


// Functions declaration
State readIncomingPacket();
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate);
static void setPositionSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw);
bool reachedTargetHeight(const float est_height, const float target);
bool targetReached(const Pose pose, const Pose target);
float collisionProbability(PacketRX *rxPacket);
float velRepulsive(PacketRX *rxPacket);
void planningTask(const Pose pose, setpoint_t *setpoint, PacketRX *rxPacket, infoPacketTX *txPacket);


// Main
void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  // Initialize packets for communication
  infoPacketTX stateTxPacket;
  stateTxPacket.message_type = 's';
  infoPacketTX taskTxPacket;
  taskTxPacket.message_type = 't';
  dataPacketTX dataTxPacket;
  dataTxPacket.message_type = 'd';

  static setpoint_t setpoint;
  State received_command = NULL;

  // Getting the Logging IDs of the state estimates
  logVarId_t idXEstimate = logGetVarId("stateEstimate", "x");
  logVarId_t idYEstimate = logGetVarId("stateEstimate", "y");
  logVarId_t idZEstimate = logGetVarId("stateEstimate", "z");
  logVarId_t idYawEstimate = logGetVarId("stateEstimate", "yaw");

  // Initialize Pose structs
  Pose poseEst;
  Pose hoveringPose = {0.0,0.0,HOVERING_HEIGHT,0.0};

  // Getting the Logging ID to see if system is tumbling or has crashed
  logVarId_t idTumble = logGetVarId("sys", "isTumbled");  // Nonzero if the system thinks it is tumbled/crashed.

  // Getting the Logging ID to see if the radio is still connected
  logVarId_t idConnected = logGetVarId("radio", "isConnected");

  // Getting Param IDs of the deck driver initialization
  //paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");

  // Other variables
  int plan_cycle = 0;
  double t_plan_avg = 0.0;


  vTaskDelay(M2T(3000));

  while(1) {
    
    // Get Pose
    poseEst.x = logGetFloat(idXEstimate);
    poseEst.y = logGetFloat(idYEstimate);
    poseEst.z = logGetFloat(idZEstimate);
    poseEst.yaw = logGetFloat(idYawEstimate);

    // Check connection
    if (logGetInt(idConnected) == 0 && state != GROUND){
      state = GROUND;
      received_command == GROUND;
      stateTxPacket.info = (float) 0;
      appchannelSendDataPacketBlock(&stateTxPacket, sizeof(stateTxPacket));
    }

    // Read incoming packet from external HW
    if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
      received_command = readIncomingPacket();
    }

    // State machine
    switch(state)
    {
      // If the drone is on the ground and receives start signal from the pc -> liftoff
      case GROUND: 
        // Stay on the ground
        //memset(&setpoint, 0, sizeof(setpoint_t));
        setPositionSetpoint(&setpoint, poseEst.x, poseEst.y, 0, poseEst.yaw);
        // Check new command
        if (received_command == HOVERING) {     // Start drone
          state = HOVERING;
          stateTxPacket.info = (float) 1;       // -> Send 1: Going toward hovering
          // appchannelSendDataPacketBlock(&stateTxPacket, sizeof(stateTxPacket));
          appchannelSendDataPacket(&stateTxPacket, sizeof(stateTxPacket));
        }
        break;

      // If the drone is hovering and receives start signal -> ready to start the task
      case HOVERING: 
        // To hovering position
        setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw);
        // Check new command
        if (received_command == GROUND || received_command == STOPPING) {  // Stop the drone
          state = STOPPING;
          stateTxPacket.info = (float) 3;     // -> Send 3: Start landing
          // Go to landing height at current xy position
          hoveringPose = poseEst;
          appchannelSendDataPacketBlock(&stateTxPacket, sizeof(stateTxPacket));
        }
        else if (received_command == READY) {     // Start the task
          if (reachedTargetHeight(poseEst.z, HOVERING_HEIGHT)) {
            state = READY;
            stateTxPacket.info = (float) 2;       // -> Send 2: Drone ready
            appchannelSendDataPacketBlock(&stateTxPacket, sizeof(stateTxPacket));
            // Start counter and statistics
            plan_cycle = 0;
            t_target_begin = clock();
          }
        }
        break;

      // If the drone is ready, go on with the task or see if stopping is needed
      case READY: 
        taskTxPacket.info = (float) 0;         // -> Task info 0: Running the task
        // Running the task
        clock_t t_plan_begin = clock();
        planningTask(poseEst, &setpoint, &rxPacket, &taskTxPacket);
        clock_t t_plan_end = clock();
        double t_plan = (double)(t_plan_end - t_plan_begin) / CLOCKS_PER_SEC;
        t_plan_avg += (t_plan - t_plan_avg)/(++plan_cycle);
        //appchannelSendDataPacketBlock(&taskTxPacket, sizeof(taskTxPacket));
        appchannelSendDataPacket(&taskTxPacket, sizeof(taskTxPacket));
        // Check if tumbling
        if (logGetInt(idTumble)){
          // Stop motors
          stateTxPacket.info = (float) 0;
          state = GROUND;
          systemRequestShutdown();
          appchannelSendDataPacketBlock(&stateTxPacket, sizeof(stateTxPacket));
        }
        // Check new command
        if (received_command == HOVERING) {   // Stop and hover
          state = HOVERING;
          stateTxPacket.info = (float) 1;     // -> Send 1: Go to hovering
          hoveringPose = poseEst;
          appchannelSendDataPacketBlock(&stateTxPacket, sizeof(stateTxPacket));
          // To hovering position
          setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw);
        }
        break;

      // If the drone is stopping -> continue descent until complete landing
      case STOPPING: 
        // Hover at landing height
        setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, landing_height, hoveringPose.yaw);
        // Check new commands
        if (received_command == GROUND) {     // Stop and land
          if (reachedTargetHeight(poseEst.z,landing_height)) {
            state = GROUND;
            stateTxPacket.info = (float) 0;   // -> Send 0: Land
            //Land
            hoveringPose = poseEst;
            appchannelSendDataPacketBlock(&stateTxPacket, sizeof(stateTxPacket));
          }
        }
        break;
    }

    vTaskDelay(M2T(19));
    commanderSetSetpoint(&setpoint, 3);

  }
}


// Functions definition

State readIncomingPacket()
{
  State received_command;
  // Read incoming packets
  if (rxPacket.info == (float) 0) { 
    received_command = GROUND;
  }
  if (rxPacket.info == (float) 1) { 
    received_command = HOVERING;
  }
  if (rxPacket.info == (float) 2) { 
    received_command = READY;
  }
  if (rxPacket.info == (float) 3) { 
    received_command = STOPPING;
  }
}

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

static void setPositionSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  
  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yaw;

  setpoint->velocity_body = true;
}

bool reachedTargetHeight(const float est_height, const float target)
{
  if ( (float)fabs(est_height - target) <= THRESH)
    return true;
  else
    return false;
}

// Functions definition for reactive planning

bool targetReached(const Pose pose, const Pose target)
{
  if ( (float)sqrt( pow(pose.x - target.x, 2) +
                    pow(pose.y - target.y, 2) +
                    pow(pose.z - target.z, 2) ) <= THRESH
        && ++target_counter >= 10)
    return true;
  else
    return false;
}

float collisionProbability(PacketRX *rxPacket)
{
  // Add safety margin along width dimension
  if (rxPacket->xmax + MARGIN <= IMG_W){
    rxPacket->xmax += MARGIN;
  } else {
    rxPacket->xmax = IMG_W;
  }
  if (rxPacket->xmin - MARGIN >= 0){
    rxPacket->xmin -= MARGIN;
  } else {
    rxPacket->xmin = 0;
  }
  
  // Get the area of the detected object
  int width = rxPacket->xmax - rxPacket->xmin;
  // int height = rxPacket->ymax - rxPacket->ymin;
  // int area = width*height;

  // Check if obstacle is on the way, if true update the collision probability
  float collision_prob = 0.0;
  if (rxPacket->xmin <= IMG_W/2 && rxPacket->xmax >= IMG_W/2) {
    // collision_prob = area/(IMG_W*IMG_H); //TODO: something related to area and object -> need to consider distance even if we don't have that info
    collision_prob = width/(IMG_W*80/100);
    if (collision_prob >= 1) {
      collision_prob = 1;
    } 
  }
  return collision_prob;
}

float velRepulsive(PacketRX *rxPacket){
  
  // Direction of the lateral repulsive velocity
  // Move in the direction opposite of the side which is occupied the most
  int dir = 1;
  // If left part of detected obstacle is bigger than right part
  if ( (IMG_W - rxPacket->xmin - rxPacket->xmax) >=0 ){   
    dir = -1;  // turn right
  } else {
    dir = 1;   // turn left
  }

  // Get the module of the lateral repulsive velocity based on the area that the object covers in the image plane
  float vel = 0.0;
  float k_vel = 1.0;
  if (dir < 0){
    vel = k_vel * (IMG_W/2 - rxPacket->xmin)/(IMG_W/2);
  } else {
    vel = k_vel * (rxPacket->xmax - IMG_W/2)/(IMG_W/2);
  }

  return dir * vel;
}


void planningTask(const Pose pose, setpoint_t *setpoint, PacketRX *rxPacket, infoPacketTX *txPacket)
{
  // Get target
  Pose target = target_points[curr_target];
  if (targetReached(pose, target)) {
    target_counter = 0;
    txPacket->info = (float) 1;   // -> Task info 1: Reached intermediate waypoint
    if (curr_target < N_WAYPOINTS-1)
      target = target_points[++curr_target];
    else
      txPacket->info = (float) 2;   // -> Task info 2: Reached final waypoint
      clock_t t_target_reached = clock();
      t_target = (double)(t_target_reached - t_target_begin) / CLOCKS_PER_SEC;
  }

  /*
  // Compute collision probability
  float safety_factor = 1.0;
  float vel_y_repulsive = 0.0;
  if (rxPacket->obj_id != (float) 0) {
    float collision_prob = collisionProbability(rxPacket);
    safety_factor = pow(collision_prob-1, 2);

    // Compute a repulsive lateral velocity to avoid the obstacle
    if (collision_prob > 0){
      vel_y_repulsive = velRepulsive(rxPacket);
    }
  }

  // DEBUG_PRINT("%.1f\n", (double) y_vel_repulsive);

  // Compute desired velocity
  float max_vel = 1.0;  // [m/s]
  float max_yaw_rate = 30.0;   // [deg/s]
  float dt = 0.1;  // [s]  time between consecutive commands
  //float vel_x = max_vel * pow(collision_prob - 1, 4) * (pow(pose.x - target.x, 2) / pow(target_points[curr_target-1].x - target.x, 2));
  //float vel_y = y_vel_repulsive + max_vel * (pow(pose.y - target.y, 2) / pow(target_points[curr_target-1].y - target.y, 2));
  //float vel_z = 0.0;
  //float vel_yaw = 0.0;

  // Compute desired velocities
  float dist_x = target.x - pose.x;
  float dist_y = target.y - pose.y;

  float rel_dist_x =  cos(pose.yaw/180*M_PI)*dist_x + sin(pose.yaw/180*M_PI)*dist_y; 
  float rel_dist_y = -sin(pose.yaw/180*M_PI)*dist_x + cos(pose.yaw/180*M_PI)*dist_y;
  float rel_yaw = atan2(rel_dist_y,rel_dist_x) * 180/M_PI;
  
  float vel_x = min(max_vel, sqrt(pow(dist_x, 2) + pow(dist_y, 2))) * safety_factor;

  float rep_yaw = atan2(vel_y_repulsive,vel_x) * 180/M_PI;

  float des_yaw_rate = (rel_yaw*safety_factor + rep_yaw)/dt;

  // apparently yaw rate command is opposite direction to the direction of change of the yaw
  float yaw_rate = 0.0;
  if (des_yaw_rate >= 0) {
    yaw_rate = max(-max_yaw_rate, -des_yaw_rate);
  } else {
    yaw_rate = min(max_yaw_rate, -des_yaw_rate);
  }
  */

  // Compute setpoint
  setPositionSetpoint(setpoint, target.x ,target.y, target.z, target.yaw);
  //TODO: switch to a velocity control for reactive collision avoidance
  //setHoverSetpoint(setpoint, vel_x, 0.0, HOVERING_HEIGHT, yaw_rate);

}



/*
typedef struct setpoint_s {
  uint32_t timestamp;

  attitude_t attitude;      // deg
  attitude_t attitudeRate;  // deg/s
  quaternion_t attitudeQuaternion;
  float thrust;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acceleration;       // m/s^2
  bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame

  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
} setpoint_t;
*/