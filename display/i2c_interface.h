/*
  display/i2c_interface.h - I2C display interface plugin

  Part of grblHAL keypad plugins

  Copyright (c) 2023 Andrew Marles

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#ifdef ARDUINO
#include "../../grbl/gcode.h"
#else
#include "grbl/gcode.h"
#endif

#include <assert.h>

typedef uint8_t msg_type_t;
typedef uint8_t machine_state_t;

enum msg_type_t {
    MachineMsg_None = 0,
// 1-128 reserved for message string length
    MachineMsg_WorkOffset = 254,
    MachineMsg_ClearMessage = 255,
};

enum machine_state_t {
    MachineState_Alarm = 1,
    MachineState_EStop = 1,
    MachineState_Cycle = 2,
    MachineState_Hold = 3,
    MachineState_ToolChange = 4,
    MachineState_Idle = 5,
    MachineState_Homing = 6,
    MachineState_Jog = 7,
    MachineState_Other = 254
};

static_assert(sizeof(msg_type_t) == 1, "msg_type_t too large for I2C display interface");
static_assert(sizeof(machine_state_t) == 1, "machine_state_t too large for I2C display interface");
static_assert(sizeof(coord_system_id_t) == 1, "coord_system_id_t too large for I2C display interface");

typedef union {
    uint8_t value;
    struct {
        uint8_t modifier :4,
                mode     :4;
    };
} jog_mode_t;

typedef union {
    float values[4];
    struct {
        float x;
        float y;
        float z;
        float a;
    };
} machine_coords_t;

typedef struct {
    uint8_t address;
    machine_state_t machine_state;
    uint8_t alarm; // -> substate?
    uint8_t home_state;
    uint8_t feed_override; // size changed in latest version!
    uint8_t spindle_override;
    uint8_t spindle_stop;
    uint8_t unused1; // -> spindle_state_t?
    int spindle_rpm;
    float feed_rate;
    coolant_state_t coolant_state;
    jog_mode_t jog_mode;
    control_signals_t signals;
    float jog_stepsize;
    coord_system_id_t current_wcs;  //active WCS or MCS modal state
    axes_signals_t limits;
    uint8_t unused2;
    uint8_t unused3;
    machine_coords_t coordinate;
    msg_type_t msgtype; //<! 1 - 128 -> msg[] contains a string msgtype long
    uint8_t msg[128];
} machine_status_packet_t;
