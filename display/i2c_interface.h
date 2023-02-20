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

typedef uint8_t machine_state_t;

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

typedef union {
    uint8_t value;
    struct {
        uint8_t modifier :4,
                mode     :4;
    };
} jog_mode_t;

// #pragma pack(push, 1) // should this struct be packed to make it (more) platform neutral?

typedef struct {
    uint8_t address;
    machine_state_t machine_state;
    uint8_t alarm;
    uint8_t home_state;
    uint8_t feed_override; // size changed in latest version!
    uint8_t spindle_override;
    uint8_t spindle_stop;
    int spindle_rpm;
    float feed_rate;
    coolant_state_t coolant_state;
    uint8_t jog_mode;  // -> jog_mode_t? includes both modifier as well as mode
    float jog_stepsize;
    coord_system_id_t current_wcs;  //active WCS or MCS modal state
    float x_coordinate;
    float y_coordinate;
    float z_coordinate;
    float a_coordinate;
} machine_status_packet_t;

// #pragma pack(pop)
