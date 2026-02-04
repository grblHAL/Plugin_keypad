/*
  display/i2c_interface.h - I2C display interface plugin

  Part of grblHAL keypad plugins

  Copyright (c) 2023 Andrew Marles

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "driver.h"

#include <assert.h>

#ifdef GRBL_ESP32
#define static_assert _Static_assert
#endif

typedef enum {
    MachineMsg_None = 0,
// 1-127 reserved for message string length
    MachineMsg_Overrides = 253,
    MachineMsg_WorkOffset = 254,
    MachineMsg_ClearMessage = 255,
} __attribute__ ((__packed__)) msg_type_t;

static_assert(sizeof(msg_type_t) == 1, "msg_type_t too large for I2C display interface");
static_assert(sizeof(system_state_t) == 1, "machine_state_t too large for I2C display interface");
static_assert(sizeof(coord_system_id_t) == 1, "coord_system_id_t too large for I2C display interface");

typedef union {
    uint8_t value;
    struct {
        uint8_t modifier :4,
                mode     :4;
    };
} jog_mode_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t diameter         :1,
                mpg              :1,
                homed            :1,
                tlo_referenced   :1,
                mode             :2, // from machine_mode_t setting
                reports_imperial :1, // $13=1
                imperial         :1; // G20 active
    };
} machine_modes_t;

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
    uint8_t version;
    system_state_t machine_state;
    uint8_t machine_substate;
    axes_signals_t home_state;
    uint8_t feed_override; // size changed in latest version!
    uint8_t spindle_override;
    uint8_t spindle_stop;
    spindle_state_t spindle_state;
    int spindle_rpm;
    float feed_rate;
    coolant_state_t coolant_state;
    jog_mode_t jog_mode;
    control_signals_t signals;
    float jog_stepsize; // always mm
    coord_system_id_t current_wcs;  // active WCS or MCS modal state
    axes_signals_t limits;
    status_code_t status_code;
    machine_modes_t machine_modes;
    machine_coords_t coordinate;
    msg_type_t msgtype; //!< 1 - 127 -> msg[] contains a string msgtype long
    uint8_t msg[128];
} machine_status_packet_t;
