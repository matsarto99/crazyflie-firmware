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
#define N_TARGETS 10

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
  char state_info;
  char task_info;
} PacketTX;

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

// SQUARE
// Pose target_points[N_TARGETS] = { // x y z yaw
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
Pose target_points[N_TARGETS] = { // x y z yaw
                                 {0.0, 0.0, HOVERING_HEIGHT, 0},
                                 {0.0, 1.0, HOVERING_HEIGHT, 0},
                                 {0.0, -1.0, HOVERING_HEIGHT, 0},
                                 {0.0, 0.0, HOVERING_HEIGHT, 0},
                                 {0.5, 0.0, HOVERING_HEIGHT, 0},
                                 {1.0, 0.0, HOVERING_HEIGHT, 0},
                                 {2.0, 0.0, HOVERING_HEIGHT, 0},
                                 {2.0, 1.0, HOVERING_HEIGHT, 0},
                                 {2.0, -1.0, HOVERING_HEIGHT, 0},
                                 {2.0, 0.0, HOVERING_HEIGHT, 0},
                                 {0.0, 0.0, HOVERING_HEIGHT, 0}
                                };

int curr_target = 0;
int target_counter = 0;


// Functions declaration
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate);
static void setPositionSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw);
bool reachedTargetHeight(const float est_height, const float target);
bool targetReached(const Pose pose, const Pose target);
float collisionProbability(PacketRX *rxPacket);
float velRepulsive(PacketRX *rxPacket, float collision_prob);
void explorationTask(const Pose pose, setpoint_t *setpoint, PacketRX *rxPacket, PacketTX *txPacket);


// Main
void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  PacketRX rxPacket;
  PacketTX txPacket;

  static setpoint_t setpoint;

  // Getting the Logging IDs of the state estimates
  logVarId_t idXEstimate = logGetVarId("stateEstimate", "x");
  logVarId_t idYEstimate = logGetVarId("stateEstimate", "y");
  logVarId_t idZEstimate = logGetVarId("stateEstimate", "z");
  logVarId_t idYawEstimate = logGetVarId("stateEstimate", "yaw");

  Pose poseEst;
  Pose hoveringPose = {0.0,0.0,HOVERING_HEIGHT,0.0};

  // Getting the Logging ID to see if system is tumbling or has crashed
  logVarId_t idTumble = logGetVarId("sys", "isTumbled");  // Nonzero if the system thinks it is tumbled/crashed.

  // Getting the Logging ID to see if the radio is still connected
  logVarId_t idConnected = logGetVarId("radio", "isConnected");

  // Getting Param IDs of the deck driver initialization
  //paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");

  vTaskDelay(M2T(3000));

  while(1) {
    
    // Get Pose
    poseEst.x = logGetFloat(idXEstimate);
    poseEst.y = logGetFloat(idYEstimate);
    poseEst.z = logGetFloat(idZEstimate);
    poseEst.yaw = logGetFloat(idYawEstimate);

    // Check connection
    if (logGetInt(idConnected) == 0 && state != GROUND){
      state = STOPPING;
      rxPacket.info = (float) 0;
    }

    // State machine
    switch(state)
    {
      // If the drone is on the ground and receives start signal from the pc -> liftoff
      case GROUND: 
        // Read incoming packets
        if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
          if (rxPacket.info == (float) 0) {               // Info 0: Do nothing
            txPacket.state_info = (float) 0;              //   -> Send 0: On the ground
            // Stay on the ground
            memset(&setpoint, 0, sizeof(setpoint_t));
          }
          else if (rxPacket.info == (float) 1) {          // Info 1: Start drone
            txPacket.state_info = (float) 1;              //   -> Send 1: Going toward hovering
            state = HOVERING;
            // Lift off
            // setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw)
          }
          appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
        }
        else {
          txPacket.state_info = (float) 0;    // If nothing is received -> Info 0: Do nothing
        }
        break;

      // If the drone is hovering and receives start signal -> ready to start the task
      case HOVERING: 
        // Read incoming packets
        if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
          if (rxPacket.info == (float) 0) {               // Info 0: Stop the drone
            txPacket.state_info = (float) 3;              //   -> Send 3: Start landing
            state = STOPPING;
            // Land
            memset(&setpoint, 0, sizeof(setpoint_t));
          }
          else if (rxPacket.info == (float) 1) {          // Info 1: Hover
            txPacket.state_info = (float) 1;              //   -> Send 1: Hovering
            // To hovering position
            // setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw)
          }
          else if (rxPacket.info == (float) 2) {            // Info 2: Start the task
            if (!reachedTargetHeight(poseEst.z, HOVERING_HEIGHT)) {
              txPacket.state_info = (float) 1;              //   -> Send 1: To hovering position
            } else {
              txPacket.state_info = (float) 2;              //   -> Send 2: Drone ready
              state = READY;
            // Still hover, start moving when in state ready
            // setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw)
            }
          }
          appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
        }
        break;

      // If the drone is ready, go on with the task or see if stopping is needed
      case READY: 
        // Read incoming packets
        if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
          if (rxPacket.info == (float) 1) {               // Info 1: Stop and hover
            txPacket.state_info = (float) 1;              //   -> Send 1: Go to hovering
            state = HOVERING;
            // Hovering at default height
            // setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            hoveringPose = poseEst
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw)
          }
          else if (rxPacket.info == (float) 2) {          // Info 2: Start/continue the task
            txPacket.state_info = (float) 2;              //   -> Send 2: Running the task
            txPacket.task_info = (float) 0;               //   -> Task info 0: Running the task
            // Running the task
            explorationTask(poseEst, &setpoint, &rxPacket, &txPacket);
            //setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            // Check if tumbling
            if (logGetInt(idTumble)){
              // Stop motors
              txPacket.state_info = (float) 0;
              state = GROUND;
              systemRequestShutdown();
            }
          }
          appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
        }
        break;

      // If the drone is stopping -> continue descent until complete landing
      case STOPPING: 
          // Read incoming packets
        if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
          if (rxPacket.info == (float) 0) {                 // Info 0: Stop and land
            if (reachedTargetHeight(poseEst.z,landing_height)) {
              txPacket.state_info = (float) 0;              //   -> Send 0: Land
              state = GROUND;
              //Land
              memset(&setpoint, 0, sizeof(setpoint_t));
            } else {
              txPacket.state_info = (float) 3;              //   -> Send 3: Lower the height before landing    
              // Decrease height
              setHoverSetpoint(&setpoint, 0, 0, landing_height, 0);
            }
          }
          appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
        }
        break;
    }

    vTaskDelay(M2T(9));
    commanderSetSetpoint(&setpoint, 3);

  }
}


// Functions definition

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
  // Add safety margin of 5px // TODO: also in y?
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
  int height = rxPacket->ymax - rxPacket->ymin;
  int area = width*height;

  // Check if obstacle is on the way, if true update the collision probability
  float collision_prob = 0.0;
  if (rxPacket->xmin <= IMG_W/2 && rxPacket->xmax >= IMG_W/2) {
    collision_prob = area/(IMG_W*IMG_H); //TODO: something related to area and object -> need to consider distance even if we don't have that info
  }

  return collision_prob;
}

float velRepulsive(PacketRX *rxPacket, float collision_prob){
  
  // Direction of the lateral repulsive velocity
  int dir = 1;
  if ( (IMG_W - rxPacket->xmin - rxPacket->xmax) >=0 ){   // move in the direction opposite of the side which is occupied the most
    dir = -1;
  } else {
    dir = 1;
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


void explorationTask(const Pose pose, setpoint_t *setpoint, PacketRX *rxPacket, PacketTX *txPacket)
{
  // Get target
  Pose target = target_points[curr_target];
  if (targetReached(pose, target)) {
    target_counter = 0;
    txPacket->task_info = (float) 1;
    if (curr_target < N_TARGETS-1)
      target = target_points[++curr_target];
    else
      txPacket->task_info = (float) 2;
  }

  // Compute collision probability
  //float collision_prob = 0.0;
  //if (rxPacket->obj_id != (float) 0) {
  //  collision_prob = collisionProbability(rxPacket);
  //}
  
  // Compute a repulsive lateral velocity to avoid the obstacle
  //float y_vel_repulsive = 0.0;
  //if (collision_prob > 0){
  //  y_vel_repulsive = velRepulsive(rxPacket, collision_prob);
  //}

  // DEBUG_PRINT("%.1f\n", (double) y_vel_repulsive);

  // Compute desired velocity
  //float max_vel = 1.0;
  //float vel_x = max_vel * pow(collision_prob - 1, 4) * (pow(pose.x - target.x, 2) / pow(target_points[curr_target-1].x - target.x, 2));
  //float vel_y = y_vel_repulsive + max_vel * (pow(pose.y - target.y, 2) / pow(target_points[curr_target-1].y - target.y, 2));
  //float vel_z = 0.0;
  //float vel_yaw = 0.0;

  // Compute setpoint
  setPositionSetpoint(setpoint, target.x ,target.y, target.z, target.yaw);
  //TODO: switch to a velocity control for reactive collision avoidance
  //setHoverSetpoint(setpoint, vel_x, vel_y, HOVERING_HEIGHT, vel_yaw);

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