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

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#define DEBUG_MODULE "OBSTACLE_AVOIDANCE"

// Custom data types

struct PacketRX {
  float info;
  float obj_id;
  float xmin;
  float ymin;
  float xmax;
  float ymax;
} __attribute__((packed));

struct PacketTX {
  char info;
} __attribute__((packed));

typedef enum {
    ground,
    hovering,
    ready,
    stopping
} State;


// Global variables
static State state = ground;
static float def_height = 0.3f;
static float landing_height = 0.15f;


// Functions declaration
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate);
bool reachedTargetHeight(float est_height,float target);
void exploration_task(setpoint_t *setpoint);


// Main
void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  struct PacketRX rxPacket;
  struct PacketTX txPacket;

  static setpoint_t setpoint;

  // Getting the Logging IDs of the state estimates
  //logVarId_t idStabilizerYaw = logGetVarId("stabilizer", "yaw");
  logVarId_t idHeightEstimate = logGetVarId("stateEstimate", "z");

  // Getting Param IDs of the deck driver initialization
  //paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");

  vTaskDelay(M2T(3000));

  while(1) {
    
    float height = logGetFloat(idHeightEstimate);

    // If the drone is on the ground and receives start signal from the pc -> liftoff
    if (state == ground) {
      // Read incoming packets
      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
        if (rxPacket.info == (float) 0) {         // Info 0: Do nothing
          txPacket.info = (float) 0;              //   -> Send 0: On the ground
          // Stay on the ground
          memset(&setpoint, 0, sizeof(setpoint_t));
        }
        else if (rxPacket.info == (float) 1) {    // Info 1: Start drone
          txPacket.info = (float) 1;              //   -> Send 1: Going toward hovering
          state = hovering;
          // Lift off
          setHoverSetpoint(&setpoint, 0, 0, def_height, 0);
        }
        appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
      }
      else {
        txPacket.info = (float) 0;    // If nothing is received -> Info 0: Do nothing
      }
      //appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
    }

    // If the drone is hovering and receives start signal -> ready to start the task
    else if (state == hovering) {
      // Read incoming packets
      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
        if (rxPacket.info == (float) 0) {         // Info 0: Stop the drone
          txPacket.info = (float) 3;              //   -> Send 3: Start landing
          state = stopping;
          // Land
          memset(&setpoint, 0, sizeof(setpoint_t));
        }
        else if (rxPacket.info == (float) 1) {    // Info 1: Hover
          txPacket.info = (float) 1;              //   -> Send 1: Hovering
          // To hovering position
          setHoverSetpoint(&setpoint, 0, 0, def_height, 0);
        }
        else if (rxPacket.info == (float) 2) {    // Info 2: Start the task
          if (!reachedTargetHeight(height,def_height)) {
            txPacket.info = (float) 1;              //   -> Send 1: To hovering position
          } else {
            txPacket.info = (float) 2;              //   -> Send 2: Drone ready
            state = ready;
          // Still hover, start moving when in state ready
          setHoverSetpoint(&setpoint, 0, 0, def_height, 0);
          }
        }
        appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
      }
      //appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
    }

    // If the drone is ready, go on with the task or see if stopping is needed
    else if (state == ready) {
      // Read incoming packets
      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
        if (rxPacket.info == (float) 1) {         // Info 1: Stop and hover
          txPacket.info = (float) 1;              //   -> Send 1: Go to hovering
          state = hovering;
          // Hovering at default height
          setHoverSetpoint(&setpoint, 0, 0, def_height, 0);
        }
        else if (rxPacket.info == (float) 2) {    // Info 2: Start/continue the task
          txPacket.info = (float) 2;              //   -> Send 2: Running the task
          // Running the task
          //setHoverSetpoint(&setpoint, velFront, velSide, def_height, 0);  // TODO: change according to target point
          setHoverSetpoint(&setpoint, 0, 0, def_height, 0);
        }
        appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
      }
      //appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
    }

    // If the drone is stopping -> continue descent until complete landing
    else if (state == stopping) {
      // Read incoming packets
      if (appchannelReceiveDataPacket(&rxPacket, sizeof(rxPacket), 1)) {
        if (rxPacket.info == (float) 0) {         // Info 0: Stop and land
          if (reachedTargetHeight(height,landing_height)) {
            txPacket.info = (float) 0;              //   -> Send 0: Land
            state = ground;
            //Land
            memset(&setpoint, 0, sizeof(setpoint_t));
          } else {
            txPacket.info = (float) 3;              //   -> Send 3: Lower the height before landing    
            // Decrease height
            setHoverSetpoint(&setpoint, 0, 0, landing_height, 0);
          }
        }
        appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
      }
      //appchannelSendDataPacketBlock(&txPacket, sizeof(txPacket));
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

bool reachedTargetHeight(float est_height,float target)
{
  if ( (float)fabs(est_height - target) <= 0.1f)
    return true;
  else
    return false;
}

void exploration_task(setpoint_t *setpoint)
{

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