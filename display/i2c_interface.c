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
#include "../keypad.h"
#endif

#include <stdio.h>
#include <string.h>

#include "i2c_interface.h"

#ifdef ARDUINO
#include "../../grbl/plugins.h"
#include "../../grbl/protocol.h"
#include "../../grbl/state_machine.h"
#else
#include "grbl/plugins.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#endif

#ifndef DISPLAY_I2CADDR
#if KEYPAD_ENABLE
#define DISPLAY_I2CADDR KEYPAD_I2CADDR
#else
#define DISPLAY_I2CADDR 0x49
#endif
#endif

static uint8_t msgtype = 0; // TODO: create a queue?
static bool send_now = false, connected = false;
static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_gcode_message_ptr on_gcode_message;
static on_wco_changed_ptr on_wco_changed;
static on_rt_reports_added_ptr on_rt_reports_added;
static on_report_handlers_init_ptr on_report_handlers_init;
static status_message_ptr status_message;
//static feedback_message_ptr feedback_message;

#if KEYPAD_ENABLE
static on_keypress_preview_ptr on_keypress_preview;
static on_jogdata_changed_ptr on_jogdata_changed;
#endif

#define SEND_STATUS_DELAY 300
#define SEND_STATUS_JOG_DELAY 100

static machine_status_packet_t status_packet, prev_status = {0};

static void send_status_info (void)
{
    uint_fast8_t idx = min(4, N_AXIS);

    system_convert_array_steps_to_mpos(status_packet.coordinate.values, sys.position);

    do {
        idx--;
        // Apply work coordinate offsets and tool length offset to current position.
        status_packet.coordinate.values[idx] -= gc_get_offset(idx);
    } while(idx);

    spindle_ptrs_t *spindle = spindle_get(0);

    status_packet.signals = hal.control.get_state();
    status_packet.limits = limit_signals_merge(hal.limits.get_state());
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

    status_packet.feed_rate = st_get_realtime_rate();

    if(msgtype || memcmp(&prev_status, &status_packet, offsetof(machine_status_packet_t, msgtype))) {

        size_t len = (status_packet.msgtype = msgtype) ? offsetof(machine_status_packet_t, msg) : offsetof(machine_status_packet_t, msgtype);

        switch(msgtype) {

            case MachineMsg_None:
            case MachineMsg_ClearMessage:
                break;

            case MachineMsg_WorkOffset:
                len += sizeof(machine_coords_t);
                break;

            case MachineMsg_Overrides:
                memcpy(status_packet.msg, &sys.override, sizeof(overrides_t));
                ((overrides_t *)status_packet.msg)->spindle_rpm = spindle->param->override_pct;
                len += sizeof(overrides_t);
                break;

            default:
                len += status_packet.msgtype;
                break;
        }

        if(i2c_send(DISPLAY_I2CADDR, (uint8_t *)&status_packet, len, false)) {
            memcpy(&prev_status, &status_packet, offsetof(machine_status_packet_t, msgtype));
            msgtype = MachineMsg_None;
        }
    }
}

static void set_state (sys_state_t state)
{
    status_packet.machine_substate = state_get_substate();

    switch (state) {
        case STATE_ESTOP:
        case STATE_ALARM:
            {
                status_packet.machine_state = MachineState_Alarm;

                char *alarm;
                if((alarm = (char *)alarms_get_description((alarm_code_t)status_packet.machine_substate))) {
                    strncpy((char *)status_packet.msg, alarm, sizeof(status_packet.msg) - 1);
                    if((alarm = strchr((char *)status_packet.msg, '.')))
                        *(++alarm) = '\0';
                    else
                        status_packet.msg[sizeof(status_packet.msg) - 1] = '\0';
                    msgtype = (msg_type_t)strlen((char *)status_packet.msg);
                }
            }
            break;
//        case STATE_ESTOP:
//            status_packet.machine_state = MachineState_EStop;
//            break;
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

    if(on_state_change)
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
    status_packet.jog_mode.mode = jogdata->mode;
    status_packet.jog_mode.modifier = jogdata->modifier_index;

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

static void onWCOChanged (void)
{
    uint_fast8_t idx = min(4, N_AXIS);
    machine_coords_t *wco = (machine_coords_t *)status_packet.msg;

    if(on_wco_changed)
        on_wco_changed();

    do {
        idx--;
        wco->values[idx] = gc_get_offset(idx);
    } while(idx);

    send_now = true;
    msgtype = MachineMsg_WorkOffset;
}

static void onGCodeMessage (char *msg)
{
    if(on_gcode_message)
        on_gcode_message(msg);

    msgtype = strlen(msg);
    if((msgtype = min(msgtype, sizeof(status_packet.msg) - 1)) == 0)
        msgtype = MachineMsg_ClearMessage; // empty string
    else
        strncpy((char *)status_packet.msg, msg, msgtype);

    send_now = true;
}

static status_code_t onStatusMessageReport (status_code_t status_code)
{
    if(status_message)
        status_code = status_message(status_code);
/*
    char *error;
    if(status_code == Status_OK && status_packet.status_code != status_code)
        msgtype = MachineMsg_ClearMessage; // empty string

    else if(status_code != Status_OK &&
             status_code != status_packet.status_code &&
              (error = (char *)errors_get_description(status_code))) {
        strncpy((char *)status_packet.msg, error, sizeof(status_packet.msg) - 1);
        if((error = strchr((char *)status_packet.msg, '.')))
            *(++error) = '\0';
        else
            status_packet.msg[sizeof(status_packet.msg) - 1] = '\0';
        msgtype = (msg_type_t)strlen((char *)status_packet.msg);
    }
*/
    status_packet.status_code = status_code;

    return status_code;
}

/*
static message_code_t onFeedbackMessageReport (message_code_t message_code)
{
    if(feedback_message)
        message_code = feedback_message(message_code);

    char *message;
    if(message_code == Message_None)
        msgtype = MachineMsg_ClearMessage; // empty string

    else if((message = (char *)message_get(message_code)->text)) {
        strncpy((char *)status_packet.msg, message, sizeof(status_packet.msg) - 1);
        status_packet.msg[sizeof(status_packet.msg) - 1] = '\0';
        msgtype = (msg_type_t)strlen((char *)status_packet.msg);
    }

    return message_code;
}
*/

static void add_reports (report_tracking_flags_t report)
{
    if(report.coolant)
        status_packet.coolant_state = hal.coolant.get_state();

    if(report.spindle) {
        spindle_ptrs_t *spindle = spindle_get(0);
        status_packet.spindle_state = spindle->get_state(spindle);
    }

    if(report.overrides) {
        spindle_ptrs_t *spindle = spindle_get(0);
        msgtype = MachineMsg_Overrides;
        status_packet.feed_override = sys.override.feed_rate > 255 ? 255 : sys.override.feed_rate;
        status_packet.spindle_override = spindle->param->override_pct > 255 ? 255 : spindle->param->override_pct;
        status_packet.spindle_stop = sys.override.spindle_stop.value;
    }

    if(report.wco)
        status_packet.current_wcs = gc_state.modal.coord_system.id;

    if(report.homed) {
        axes_signals_t homing = {sys.homing.mask ? sys.homing.mask : AXES_BITMASK};
        status_packet.home_state.mask = sys.homing.mask & sys.homed.mask;
        status_packet.machine_modes.homed = (homing.mask & sys.homed.mask) == homing.mask;
    }

    if(report.tlo_reference)
        status_packet.machine_modes.tlo_referenced = sys.tlo_reference_set.mask != 0;

    if(report.xmode)
        status_packet.machine_modes.diameter = gc_state.modal.diameter_mode;

    if(report.mpg_mode)
        status_packet.machine_modes.mpg = sys.mpg_mode;
}

static void onRealtimeReportsAdded (report_tracking_flags_t report)
{
    if(on_rt_reports_added)
        on_rt_reports_added(report);

    add_reports(report);
}

static void onReportHandlersInit (void)
{
    if(on_report_handlers_init)
        on_report_handlers_init();

    status_message = grbl.report.status_message;
    grbl.report.status_message = onStatusMessageReport;
/*
    feedback_message = grbl.report.feedback_message;
    grbl.report.feedback_message = onFeedbackMessageReport;
 */
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write(connected ? "[PLUGIN:I2C Display v0.09]" ASCII_EOL : "[PLUGIN:I2C Display v0.09 (not connected)]" ASCII_EOL);
}

static void complete_setup (void *data)
{
    report_tracking_flags_t report = {
        .coolant = On,
        .spindle = On,
        .overrides = On,
        .homed = On,
        .xmode = On,
        .mpg_mode = On,
        .wco = On
    };

    set_state(state_get());
    add_reports(report);

    status_packet.machine_modes.mode = settings.mode;

    on_execute_delay = grbl.on_execute_delay;
    grbl.on_execute_delay = display_poll_delay;
}

void display_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    hal.delay_ms(510, NULL);

    if((connected = i2c_probe(DISPLAY_I2CADDR))) {

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = display_poll_realtime;

        on_state_change = grbl.on_state_change;
        grbl.on_state_change = onStateChanged;

        on_wco_changed = grbl.on_wco_changed;
        grbl.on_wco_changed = onWCOChanged;

        on_gcode_message = grbl.on_gcode_message;
        grbl.on_gcode_message = onGCodeMessage;

        on_report_handlers_init = grbl.on_report_handlers_init;
        grbl.on_report_handlers_init = onReportHandlersInit;

        on_rt_reports_added = grbl.on_rt_reports_added;
        grbl.on_rt_reports_added = onRealtimeReportsAdded;

        status_packet.address = 0x01;
        status_packet.msgtype = MachineMsg_None;
        status_packet.status_code = Status_OK;
    #if N_AXIS == 3
        status_packet.coordinate.a = 0xFFFFFFFF;
    #endif

        // delay final setup until startup is complete
        protocol_enqueue_foreground_task(complete_setup, NULL);

#if KEYPAD_ENABLE

        on_keypress_preview = keypad.on_keypress_preview;
        keypad.on_keypress_preview = keypress_preview;

        on_jogdata_changed = keypad.on_jogdata_changed;
        keypad.on_jogdata_changed = jogdata_changed;

#endif

    } else
        protocol_enqueue_foreground_task(report_warning, "I2C display not connected!");
}

#endif // DISPLAY_ENABLE
