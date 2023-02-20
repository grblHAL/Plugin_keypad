/*
  display/i2c_interface.c - I2C display interface plugin

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


#ifdef ARDUINO
#include "../../driver.h"
#else
#include "driver.h"
#endif

#if DISPLAY_ENABLE == 1

#if KEYPAD_ENABLE
#include "keypad.h"
#endif

#include <stdio.h>
#include <string.h>

#include "i2c_interface.h"

#ifdef ARDUINO
#include "../../i2c.h"
#include "../../grbl/report.h"
#include "../../grbl/override.h"
#include "../../grbl/protocol.h"
#include "../../grbl/state_machine.h"
#else
#include "i2c.h"
#include "grbl/report.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/machine_limits.h"
#include "grbl/state_machine.h"
#endif

#ifndef DISPLAY_I2CADDR
#if KEYPAD_ENABLE
#define DISPLAY_I2CADDR KEYPAD_I2CADDR
#else
#define DISPLAY_I2CADDR 0x49
#endif
#endif

static bool send_now = false;
static on_state_change_ptr on_state_change;
static on_override_changed_ptr on_override_changed;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
#if KEYPAD_ENABLE
static on_keypress_preview_ptr on_keypress_preview;
static on_jogdata_changed_ptr on_jogdata_changed;
#endif

#define SEND_STATUS_DELAY 300
#define SEND_STATUS_JOG_DELAY 100

static machine_status_packet_t status_packet, prev_status = {0};

static void send_status_info (void)
{
    int32_t current_position[N_AXIS]; // Copy current state of the system position variable
    float print_position[N_AXIS];

    memcpy(current_position, sys.position, sizeof(sys.position));

    system_convert_array_steps_to_mpos(print_position, current_position);

    uint_fast8_t idx;
    float wco[N_AXIS];
    for (idx = 0; idx < N_AXIS; idx++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[idx] = gc_get_offset(idx);
        print_position[idx] -= wco[idx];
    }

    spindle_ptrs_t *spindle = spindle_get(0);

    status_packet.coolant_state = hal.coolant.get_state();
    status_packet.feed_override = sys.override.feed_rate > 255 ? 255 : sys.override.feed_rate;
    status_packet.spindle_override = spindle->param->override_pct > 255 ? 255 : spindle->param->override_pct;
    status_packet.spindle_stop = sys.override.spindle_stop.value;
/*
    // Report realtime feed speed
    if(spindle->cap.variable) {
        status_packet.spindle_rpm = spindle->param->rpm_overridden;
        if(spindle->get_data)
            status_packet.spindle_rpm = spindle->get_data(SpindleData_RPM)->rpm;
    } else
        status_packet.spindle_rpm = spindle->param->rpm;
*/
    status_packet.spindle_rpm = spindle->param->rpm_overridden;  //rpm should be changed to actual reading
    status_packet.home_state = (uint8_t)(sys.homing.mask & sys.homed.mask);
    status_packet.x_coordinate = print_position[0];
    status_packet.y_coordinate = print_position[1];
    status_packet.z_coordinate = print_position[2];
#if N_AXIS > 3
    status_packet.a_coordinate = print_position[3];
#endif

    status_packet.feed_rate = st_get_realtime_rate();
    status_packet.current_wcs = gc_state.modal.coord_system.id;

    if(memcmp(&prev_status, &status_packet, sizeof(machine_status_packet_t))) {
        if(I2C_Send(DISPLAY_I2CADDR, (uint8_t *)&status_packet, sizeof(machine_status_packet_t), false))
            memcpy(&prev_status, &status_packet, sizeof(machine_status_packet_t));
    }
}

static void set_state (sys_state_t state)
{
    status_packet.alarm = (uint8_t)sys.alarm;

    switch (state) {
        case STATE_ALARM:
            status_packet.machine_state = MachineState_Alarm;
            break;
        case STATE_ESTOP:
            status_packet.machine_state = MachineState_EStop;
            break;
        case STATE_CYCLE:
            status_packet.machine_state = MachineState_Cycle;
            break;
        case STATE_HOLD:
            status_packet.machine_state = MachineState_Hold;
            break;
        case STATE_TOOL_CHANGE:
            status_packet.machine_state = MachineState_ToolChange;
            break;
        case STATE_IDLE:
            status_packet.machine_state = MachineState_Idle;
            break;
        case STATE_HOMING:
            status_packet.machine_state = MachineState_Homing;
            break;
        case STATE_JOG:
            status_packet.machine_state = MachineState_Jog;
            break;
        default:
            status_packet.machine_state = MachineState_Other;
            break;
    }
}

static void onStateChanged (sys_state_t state)
{
    set_state(state);
    send_now = true;

    if(on_state_change)         // Call previous function in the chain.
        on_state_change(state);
}

static void display_poll (sys_state_t state)
{
    static uint32_t last_ms;

    uint32_t ms = hal.get_elapsed_ticks();

    // don't spam the port - wait at least 10ms after previous send
    if(send_now && ms - last_ms >= 10) {
        send_status_info();
        last_ms = ms;
        send_now = false;
        return;
    }

    //check more often during manual jogging
    if(ms - last_ms >= (state == STATE_JOG ? SEND_STATUS_JOG_DELAY : SEND_STATUS_DELAY)) {
        send_status_info();
        last_ms = ms;
    }
}

static void display_poll_realtime (sys_state_t state)
{
    on_execute_realtime(state);
    display_poll(state);
}

static void display_poll_delay (sys_state_t state)
{
    on_execute_delay(state);
    display_poll(state);
}

#if KEYPAD_ENABLE

static bool keypress_preview (char keycode, sys_state_t state)
{
    switch(keycode) {

        case '?':                                    // pendant attach
        case CMD_SAFETY_DOOR:
        case CMD_OPTIONAL_STOP_TOGGLE:
        case CMD_SINGLE_BLOCK_TOGGLE:
        case CMD_PROBE_CONNECTED_TOGGLE:
            send_now = true;
            break;
    }

    if(on_keypress_preview)
        return on_keypress_preview(keycode, state);

    return false;
}

static void jogdata_changed (jogdata_t *jogdata)
{
    status_packet.jog_mode = ((uint8_t)jogdata->mode << 4) | (uint8_t)jogdata->modifier_index;

    switch(jogdata->mode){

        case JogMode_Slow:
            status_packet.jog_stepsize = jogdata->settings.slow_speed * jogdata->modifier[jogdata->modifier_index];
            break;

        case JogMode_Fast:
            status_packet.jog_stepsize = jogdata->settings.fast_speed * jogdata->modifier[jogdata->modifier_index];
            break;

        default:
            status_packet.jog_stepsize = jogdata->settings.step_distance * jogdata->modifier[jogdata->modifier_index];
            break;
    }

    send_now = true;
}

#endif

static void override_changed (override_changed_t override)
{
    send_now = true;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:I2C Display v0.01]" ASCII_EOL);
}

void display_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = display_poll_realtime;

    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = display_poll_delay;

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = onStateChanged;

    on_override_changed = grbl.on_override_changed;
    grbl.on_override_changed = override_changed;

    status_packet.address = 0x01;
#if N_AXIS == 3
    status_packet.a_coordinate = 0xFFFFFFFF;
#endif

    set_state(state_get());

#if KEYPAD_ENABLE

    on_keypress_preview = keypad.on_keypress_preview;
    keypad.on_keypress_preview = keypress_preview;

    on_jogdata_changed = keypad.on_jogdata_changed;
    keypad.on_jogdata_changed = jogdata_changed;

#endif
}

#endif

