/**
 * Author: Mattia Sartori
 * E-mail: matsarto99@gmail.com
 *
 * Developed in the context of the author's Master's Thesis project
 * 
 * Code for reactive planning for obstacle avoidance on the Bitcraze Crazyflie 2.1 drone.
 * - The code should run in parallel with the wifi image streamer on the AIdeck.
 * - The images are streames to an external pc which is connected to the drone via radio link.
 * - The pc runs an object detection task and send the bounding boxes to the drone.
 * - The drone plans its commands to reach a desired set of waypoints
 * - If an obstacle is detected, the drone plans a reactive command in order to avoid it and reach the waypoint safely
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

#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) > (b)) ? (a) : (b))

#define DEBUG_MODULE "OBSTACLE_AVOIDANCE"
#define HOVERING_HEIGHT 0.4f        // Default hovering height
#define LANDING_HEIGHT 0.1f         // Landing height to be reached before stopping the motors
#define TARGET_THRESHOLD 0.15f      // Minimum distance to target for which the waypoint is considered to be reached
#define TARGET_COUNTER 5            // Number of consecutive iterations in which the drone has to be inside the target threshold
                                    //    before switching to the next target
#define IMG_W 320                   // Image width
#define IMG_H 240                   // Image height
#define MARGIN 20                   // Safety margin in pixel added to the bounding box of the detcted obstacle
#define N_WAYPOINTS 2               // Number of waypoints to traverse

float PI = 3.14159265358979323846;

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
  float task_info;
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
int curr_target = 0;
int target_counter = 0;
long t_target_begin;
long t_target;
float safety_factor = 1.0;
float vel_y_repulsive = 0.0;

// Sequence of targets
// positive yaw -> turn left

// SQUARE
// Pose target_points[N_WAYPOINTS] = { // x y z yaw
//                                    {0.0, 0.0, HOVERING_HEIGHT, 0},
//                                    {0.5, 0.0, HOVERING_HEIGHT, 0},
//                                    {0.5, 0.0, HOVERING_HEIGHT, 90},
//                                    {0.5, 0.5, HOVERING_HEIGHT, 90},
//                                    {0.5, 0.5, HOVERING_HEIGHT, 180},
//                                    {0.0, 0.5, HOVERING_HEIGHT, 180},
//                                    {0.0, 0.5, HOVERING_HEIGHT, 270},
//                                    {0.0, 0.0, HOVERING_HEIGHT, 270},
//                                    {0.0, 0.0, HOVERING_HEIGHT, 360},
//                                    {0.0, 0.0, HOVERING_HEIGHT, 0} 
//                                   };

// LEFT, RIGHT, FORTH, LEFT, RIGHT, BACK
//Pose target_points[N_WAYPOINTS] = { // x y z yaw
//                                   {0.0, 0.0, HOVERING_HEIGHT, 0},
//                                   {0.0, 0.5, HOVERING_HEIGHT, 0},
//                                   {0.0, -0.5, HOVERING_HEIGHT, 0},
//                                   {0.0, 0.0, HOVERING_HEIGHT, 0},
//                                   {0.5, 0.0, HOVERING_HEIGHT, 0},
//                                   {1.0, 0.0, HOVERING_HEIGHT, 0},
//                                   {1.0, 0.5, HOVERING_HEIGHT, 0},
//                                   {1.0, -0.5, HOVERING_HEIGHT, 0},
//                                   {1.0, 0.0, HOVERING_HEIGHT, 0},
//                                   {0.0, 0.0, HOVERING_HEIGHT, 0}
//                                  };

// Straight path, supposed to be with an obstacle on the way
Pose target_points[N_WAYPOINTS] = { // x y z yaw
                                   {0.0, 0.0, HOVERING_HEIGHT, 0},
                                   {3.5, 0.0, HOVERING_HEIGHT, 0}
                                  };


// Functions declaration

/**
 * \brief Set a command in terms of desired velocities in body frame and desired absolute height
 * \param setpoint Pointer to the setpoint for setting commands
 * \param vx Desired velocity along x
 * \param vy Desired velocity along y
 * \param z Desired height
 * \param yawrate Desired yaw rate
 */
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate);

/**
 * \brief Set a command in terms of absolute position and attitude
 * \param setpoint Pointer to the setpoint for setting commands
 * \param x Desired position x
 * \param y Desired position y
 * \param z Desired height
 * \param yaw Desired yaw
 */
static void setPositionSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw);

/**
 * \brief Check if the drone reached the desired height
 * \param est_height Height estimated by the drone
 * \param des_height Desired height to reach
 */
bool reachedTargetHeight(const float est_height, const float des_height);

/**
 * \brief Check if the drone reached the desired target
 * \param pose Estimated pose
 * \param target Desired target
 */
bool targetReached(const Pose pose, const Pose target);

/**
 * \brief Compute the collision risk based on the boundung box parameters of the detected obstacle
 * \param rxPacket Pointer to the received packet containing the bbox
 */
float collisionRisk(PacketRX *rxPacket);

/**
 * \brief Compute the repulsive action when an obstacle is on the way
 * \param rxPacket Pointer to the received packet containing the bbox
 */
float velRepulsive(PacketRX *rxPacket);

/**
 * \brief Run the reactive planner, compute commands to avoid the obstacle
 * \param pose Estimated pose
 * \param setpoint Pointer to the setpoint for setting commands
 * \param rxPacket Pointer to the received packet containing the bbox
 * \param txPacket Pointer to the packet to transmit
 */
void planningTask(const Pose pose, setpoint_t *setpoint, PacketRX *rxPacket, PacketTX *txPacket);


// Main
void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  // Initialize packets for communication
  PacketRX rxPacket;
  PacketTX txPacket;

  // Setpoint to define desired commands
  static setpoint_t setpoint;

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

  // Other variables
  int plan_cycle = 0;
  double t_plan_avg = 0.0;

  vTaskDelay(M2T(3000));

  // State machine
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
        if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 10)) {
          if (rxPacket.info == (float) 0) {               // Info 0: Do nothing
            txPacket.state_info = (float) 0;              //   -> Send 0: On the ground
            // Stay on the ground
            memset(&setpoint, 0, sizeof(setpoint_t));
            // setPositionSetpoint(&setpoint, poseEst.x, poseEst.y, 0, 0);
            // setHoverSetpoint(&setpoint, 0, 0, 0, 0);
          }
          else if (rxPacket.info == (float) 1) {          // Info 1: Start drone
            txPacket.state_info = (float) 1;              //   -> Send 1: Going toward hovering
            state = HOVERING;
            // Lift off
            // setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw);
          }
          appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
          //commanderSetSetpoint(&setpoint, 3);
        }
        else {
          txPacket.state_info = (float) 0;    // If nothing is received -> Info 0: Do nothing
        }
        break;

      // If the drone is hovering and receives start signal -> ready to start the task
      case HOVERING: 
        // Read incoming packets
        if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 10)) {
          if (rxPacket.info == (float) 0) {               // Info 0: Stop the drone
            txPacket.state_info = (float) 3;              //   -> Send 3: Start landing
            state = STOPPING;
            // Land
            // memset(&setpoint, 0, sizeof(setpoint_t));
            hoveringPose = poseEst;
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, LANDING_HEIGHT, hoveringPose.yaw);
          }
          else if (rxPacket.info == (float) 1) {          // Info 1: Hover
            txPacket.state_info = (float) 1;              //   -> Send 1: Hovering
            // To hovering position
            // setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw);
          }
          else if (rxPacket.info == (float) 2) {            // Info 2: Start the task
            if (!reachedTargetHeight(poseEst.z, HOVERING_HEIGHT)) {
              txPacket.state_info = (float) 1;              //   -> Send 1: To hovering position
            } else {
              txPacket.state_info = (float) 2;              //   -> Send 2: Drone ready
              txPacket.task_info = (float) 0;               //   -> Task info 0: Running the task
              state = READY;
              plan_cycle = 0;
              t_target_begin = xTaskGetTickCount();
            // Still hover, start moving when in state ready
            // setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw);
            }
          }
          appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
          //commanderSetSetpoint(&setpoint, 3);
        }
        break;

      // If the drone is ready, go on with the task or see if stopping is needed
      case READY: 
        // Read incoming packets
        if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 10)) {
          if (rxPacket.info == (float) 1) {               // Info 1: Stop and hover
            txPacket.state_info = (float) 1;              //   -> Send 1: Go to hovering
            state = HOVERING;
            // Hovering at default height in the current position
            // setHoverSetpoint(&setpoint, 0, 0, HOVERING_HEIGHT, 0);
            hoveringPose = poseEst;
            setPositionSetpoint(&setpoint, hoveringPose.x, hoveringPose.y, hoveringPose.z, hoveringPose.yaw);
          }
          else if (rxPacket.info == (float) 2) {          // Info 2: Start/continue the task
            txPacket.state_info = (float) 2;              //   -> Send 2: Running the task
            txPacket.task_info = (float) 0;               //   -> Task info 0: Running the task
            long t_plan_begin = xTaskGetTickCount();
            
            // Running the task
            planningTask(poseEst, &setpoint, &rxPacket, &txPacket);
            
            long t_plan_end = xTaskGetTickCount();
            double t_plan = (double)(t_plan_end - t_plan_begin);
            t_plan_avg += (t_plan - t_plan_avg)/(++plan_cycle);

            // Check if tumbling
            if (logGetInt(idTumble)){
              // Stop motors
              txPacket.state_info = (float) 0;
              state = GROUND;
              systemRequestShutdown();
            }
          }
          appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
          //commanderSetSetpoint(&setpoint, 3);
        }
        break;

      // If the drone is stopping -> continue descent until complete landing
      case STOPPING: 
          // Read incoming packets
        if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 10)) {
          if (rxPacket.info == (float) 0) {                 // Info 0: Stop and land
            if (reachedTargetHeight(poseEst.z,LANDING_HEIGHT)) {
              txPacket.state_info = (float) 0;              //   -> Send 0: Land
              state = GROUND;
              //Land
              memset(&setpoint, 0, sizeof(setpoint_t));
              // setPositionSetpoint(&setpoint, poseEst.x, poseEst.y, 0, 0);
              // setHoverSetpoint(&setpoint, 0, 0, 0, 0);
            } else {
              txPacket.state_info = (float) 3;              //   -> Send 3: Lower the height before landing    
              // Decrease height
              setHoverSetpoint(&setpoint, 0, 0, LANDING_HEIGHT, 0);
            }
          }
          appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
          //commanderSetSetpoint(&setpoint, 3);
        }
        break;
    }

    commanderSetSetpoint(&setpoint, 3);
    vTaskDelay(M2T(50));

  }
}


// Functions definition

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;   // m

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;   // deg/s

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;    // m/s
  setpoint->velocity.y = vy;    // m/s

  setpoint->velocity_body = true;   // true if velocity is given in body frame; false if velocity is given in world frame
}

static void setPositionSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  
  setpoint->position.x = x;   // m
  setpoint->position.y = y;   // m
  setpoint->position.z = z;   // m

  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yaw;   // deg

  setpoint->velocity_body = true;   // true if velocity is given in body frame; false if velocity is given in world frame
}

bool reachedTargetHeight(const float est_height, const float des_height)
{
  if ( (float)fabs(est_height - des_height) <= TARGET_THRESHOLD)
    return true;
  else
    return false;
}

bool targetReached(const Pose pose, const Pose target)
{
  return ( (float)sqrt( pow(pose.x - target.x, 2) +
                        pow(pose.y - target.y, 2) +
                        pow(pose.z - target.z, 2) ) <= TARGET_THRESHOLD);
}

float collisionRisk(PacketRX *rxPacket)
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

  // Check if obstacle is on the way, if true, update the collision risk
  float collision_risk = 0.0;
  if (rxPacket->xmin <= IMG_W/2 && rxPacket->xmax >= IMG_W/2) {
    //Collision risk related to area and object -> need to consider distance even if we don't have that info
    // collision_risk = area/(IMG_W*IMG_H); 
    collision_risk = (float) width/(IMG_W*80/100);
    if (collision_risk >= 1) {
      collision_risk = 1;
    }
  }
  return collision_risk;
}

float velRepulsive(PacketRX *rxPacket){
  
  // Direction of the lateral repulsive velocity
  // Move in the direction opposite of the side which is occupied the most
  int dir = 1;
  // If the object is mainly in the left side of the image plane ? turn right : turn left
  if ( (IMG_W - rxPacket->xmin - rxPacket->xmax) >=0 ){   
    dir = -1;  // turn right
  } else {
    dir = 1;   // turn left
  }

  // Get the module of the lateral repulsive velocity based on the area that the object covers in the image plane
  float vel = 0.0;
  float k_vel = 0.6;
  if (dir < 0){
    vel = k_vel * (IMG_W/2 - rxPacket->xmin)/(IMG_W/2);
  } else {
    vel = k_vel * (rxPacket->xmax - IMG_W/2)/(IMG_W/2);
  }

  return dir * vel;
}

void planningTask(const Pose pose, setpoint_t *setpoint, PacketRX *rxPacket, PacketTX *txPacket)
{
  float max_vel = 1.0;  // [m/s]
  float max_yaw_rate = 60.0;   // [deg/s]
  float dt = 0.1;  // [s]  time between consecutive commands
  float alpha = 0.5;
  float beta = 0.5;
  
  // Get target
  Pose target = target_points[curr_target];
  
  // If target is reached, move to the next one
  // If close to target, send command to stay over the target for some consecutive iterations
  if (targetReached(pose, target)) {
    float dist_x = target.x - pose.x;
    float dist_y = target.y - pose.y;

    float rel_dist_x =  cosf(pose.yaw/180*PI)*dist_x + sinf(pose.yaw/180*PI)*dist_y; 
    float rel_dist_y = -sinf(pose.yaw/180*PI)*dist_x + cosf(pose.yaw/180*PI)*dist_y;

    float dist_yaw = target.yaw-pose.yaw;
    float yr = 0.0;
    if (dist_yaw >= 0) {
      yr = min(max_yaw_rate, dist_yaw/dt);
    } else {
      yr = max(-max_yaw_rate, dist_yaw/dt);
    }

    // If the drone is in reach of the target, command a movement over the target
    setHoverSetpoint(setpoint, rel_dist_x, rel_dist_y, target.z, yr);

    // Stay on the target for some consecutive iterations
    if (++target_counter >= TARGET_COUNTER){
      // Target has been "reliably" reached
      long t_target_reached = xTaskGetTickCount();
      t_target = (double)(t_target_reached - t_target_begin);
      target_counter = 0;
      txPacket->task_info = (float) 1;  // Reached intermediate waypoint
      if (curr_target < N_WAYPOINTS-1) {
        // Update target waypoint
        target = target_points[++curr_target];
      } else {
        txPacket->task_info = (float) 2;  // Reached final waypoint
        state = HOVERING;
      }
    }
    return;
  }

  // Compute collision risk and repulsive action
  float new_factor = 1.0;
  float new_vel_y_repulsive = 0.0;
  if (rxPacket->obj_id != (float) 0) {
    float collision_risk = collisionRisk(rxPacket);
    new_factor = powf(collision_risk-1, 2);

    // Compute a repulsive lateral velocity to avoid the obstacle
    if (collision_risk > 0){
      new_vel_y_repulsive = velRepulsive(rxPacket);
      // DEBUG_PRINT("%.1f\n", (double) y_vel_repulsive);
    }
  }

  // Introduce memory in the repulsive action and in the safety factor
  vel_y_repulsive = vel_y_repulsive * (1-beta) + new_vel_y_repulsive * beta;
  safety_factor = safety_factor * (1-alpha) + new_factor * alpha;

  // Distance in global reference frame
  float dist_x = target.x - pose.x;
  float dist_y = target.y - pose.y;

  // Distances and angle offset in relative reference frame
  float rel_dist_x =  cosf(pose.yaw/180*PI)*dist_x + sinf(pose.yaw/180*PI)*dist_y; 
  float rel_dist_y = -sinf(pose.yaw/180*PI)*dist_x + cosf(pose.yaw/180*PI)*dist_y;
  float rel_yaw = atan2f(rel_dist_y,rel_dist_x) * 180/PI;
  
  // Compute desired velocities
  // *** Considering obstacle ***
  float vel_x = min(max_vel, sqrtf(powf(dist_x, 2) + powf(dist_y, 2))) * safety_factor * fabsf(rel_yaw/180 - 1);
  float rep_yaw = atan2f(vel_y_repulsive,vel_x) * 180/PI;
  float des_yaw_rate = (rel_yaw*fabsf(rel_yaw/180 - 1)*((float) 0.8) + rep_yaw)*safety_factor/dt;
  
  // Yaw rate command commanded from the drone is in the same direction to the direction of change of the yaw
  // It is not true when commands are sent from the pc
  float yaw_rate = 0.0;
  if (des_yaw_rate >= 0) {
    yaw_rate = min(max_yaw_rate, des_yaw_rate);
  } else {
    yaw_rate = max(-max_yaw_rate, des_yaw_rate);
  }

  // Compute desired velocities
  // *** Without considering obstacle ***
  // float vel_x = min(max_vel, sqrtf(powf(dist_x, 2) + powf(dist_y, 2))) * fabsf(rel_yaw/180 - 1);
  // float yaw_rate = 0.0;
  // if (rel_yaw >= 0) {
  //   yaw_rate = min(max_yaw_rate, rel_yaw/dt);
  // } else {
  //   yaw_rate = max(-max_yaw_rate, rel_yaw/dt);
  // }

  // Compute setpoint
  setHoverSetpoint(setpoint, vel_x, 0.0, HOVERING_HEIGHT, yaw_rate);

}

