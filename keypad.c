/*
  keypad.c - I2C keypad plugin

  Part of grblHAL

  Copyright (c) 2017-2022 Terje Io

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
#include "../driver.h"
#else
#include "driver.h"
#endif

#if KEYPAD_ENABLE

#include <stdio.h>
#include <string.h>

#include "keypad.h"

#ifdef ARDUINO
#include "../i2c.h"
#include "../grbl/report.h"
#include "../grbl/override.h"
#include "../grbl/protocol.h"
#include "../grbl/nvs_buffer.h"
#include "../grbl/state_machine.h"
#else
#include "i2c.h"
#include "grbl/report.h"
#include "grbl/override.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/state_machine.h"
#include "grbl/limits.h"
#endif

typedef struct {
    char buf[KEYBUF_SIZE];
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
} keybuffer_t;

static char buf[(STRLEN_COORDVALUE + 1) * N_AXIS];

static on_state_change_ptr on_state_change;
//static on_execute_realtime_ptr on_execute_realtime; // For real time loop insertion

#define SEND_STATUS_DELAY 300

static bool is_executing = false;
static char *command;
static nvs_address_t nvs_address;
static macro_settings_t plugin_settings;
static stream_read_ptr stream_read;
static driver_reset_ptr driver_reset;

static int16_t get_macro_char (void);

Machine_status_packet status_packet;

uint8_t *status_ptr = (uint8_t*) &status_packet;
static bool jogging = false, keyreleased = true;
static jogmode_t jogMode = JogMode_Fast;
static jogmodify_t jogModify = JogModify_1;
static jog_settings_t jog;
static keybuffer_t keybuf = {0};
static uint32_t nvs_address;
static on_report_options_ptr on_report_options;
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;
static on_realtime_report_ptr on_realtime_report;
static on_jogmode_changed_ptr on_jogmode_changed;
static on_jogmodify_changed_ptr on_jogmodify_changed;

keypad_t keypad = {0};

static const setting_detail_t keypad_settings[] = {
    { Setting_JogStepSpeed, Group_Jogging, "Step jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.step_speed, NULL, NULL },
    { Setting_JogSlowSpeed, Group_Jogging, "Slow jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.slow_speed, NULL, NULL },
    { Setting_JogFastSpeed, Group_Jogging, "Fast jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.fast_speed, NULL, NULL },
    { Setting_JogStepDistance, Group_Jogging, "Step jog distance", "mm", Format_Decimal, "#0.000", NULL, NULL, Setting_NonCore, &jog.step_distance, NULL, NULL },
    { Setting_JogSlowDistance, Group_Jogging, "Slow jog distance", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.slow_distance, NULL, NULL },
    { Setting_JogFastDistance, Group_Jogging, "Fast jog distance", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.fast_distance, NULL, NULL },
    { Setting_UserDefined_0, Group_UserSettings, "Macro 1 UP", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[0].data, NULL, NULL },
    { Setting_UserDefined_1, Group_UserSettings, "Macro 2 RIGHT", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[1].data, NULL, NULL },
    { Setting_UserDefined_2, Group_UserSettings, "Macro 3 DOWN", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[2].data, NULL, NULL },
    { Setting_UserDefined_3, Group_UserSettings, "Macro 4 LEFT", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[3].data, NULL, NULL },
    { Setting_UserDefined_4, Group_UserSettings, "Macro 5 SPINDLE", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[4].data, NULL, NULL },
#if N_MACROS > 5
    { Setting_UserDefined_5, Group_UserSettings, "Macro 5 RAISE", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[5].data, NULL, NULL },
    { Setting_UserDefined_6, Group_UserSettings, "Macro 6 LOWER", NULL, Format_String, "x(127)", "0", "127", Setting_NonCore, &plugin_settings.macro[6].data, NULL, NULL },
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t keypad_settings_descr[] = {
    { Setting_JogStepSpeed, "Step jogging speed in millimeters per minute." },
    { Setting_JogSlowSpeed, "Slow jogging speed in millimeters per minute." },
    { Setting_JogFastSpeed, "Fast jogging speed in millimeters per minute." },
    { Setting_JogStepDistance, "Jog distance for single step jogging." },
    { Setting_JogSlowDistance, "Jog distance before automatic stop." },
    { Setting_JogFastDistance, "Jog distance before automatic stop." },
    { Setting_UserDefined_0, "Macro content for macro 1, separate blocks (lines) with the vertical bar character |." },
    { Setting_UserDefined_1, "Macro content for macro 2, separate blocks (lines) with the vertical bar character |." },
    { Setting_UserDefined_2, "Macro content for macro 3, separate blocks (lines) with the vertical bar character |." },
    { Setting_UserDefined_3, "Macro content for macro 4, separate blocks (lines) with the vertical bar character |." },
    { Setting_UserDefined_4, "Spindle Macro.  Use to start spindle, or turn it off if running." },
#if N_MACROS > 5
    { Setting_UserDefined_5, "Macro content for macro 4, separate blocks (lines) with the vertical bar character |." },
    { Setting_UserDefined_6, "Macro content for macro 4, separate blocks (lines) with the vertical bar character |." },
#endif    
};

#endif

// Ends macro execution if currently running
// and restores normal operation.
static void end_macro (void)
{
    is_executing = false;
    if(hal.stream.read == get_macro_char) {
        hal.stream.read = stream_read;
        report_init_fns();
    }
}

// Called on a soft reset so that normal operation can be restored.
static void plugin_reset (void)
{
    end_macro();    // End macro if currently running.
    driver_reset(); // Call the next reset handler in the chain.
}

// Macro stream input function.
// Reads character by character from the macro and returns them when
// requested by the foreground process.
static int16_t get_macro_char (void)
{
    static bool eol_ok = false;

    if(*command == '\0') {                          // End of macro?
        end_macro();                                // If end reading from it
        return eol_ok ? SERIAL_NO_DATA : ASCII_LF;  // and return a linefeed if the last character was not a linefeed.
    }

    char c = *command++;    // Get next character.

    if((eol_ok = c == '|')) // If character is vertical bar
        c = ASCII_LF;       // return a linefeed character.

    return (uint16_t)c;
}

// This code will be executed after each command is sent to the parser,
// If an error is detected macro execution will be stopped and the status_code reported.
static status_code_t trap_status_report (status_code_t status_code)
{
    if(status_code != Status_OK) {
        char msg[30];
        sprintf(msg, "error %d in macro", (uint8_t)status_code);
        report_message(msg, Message_Warning);
        end_macro();
    }

    return status_code;
}

// Actual start of macro execution.
static void run_macro (uint_fast16_t state)
{
    if(state == STATE_IDLE && hal.stream.read != get_macro_char) {
        stream_read = hal.stream.read;                      // Redirect input stream to read from the macro instead of
        hal.stream.read = get_macro_char;                   // the active stream. This ensures that input streams are not mingled.
        grbl.report.status_message = trap_status_report;    // Add trap for status messages so we can terminate on errors.
    }
}

// On falling interrupt run macro if machine is in Idle state.
// Since this function runs in an interrupt context actual start of execution
// is registered as a single run task to be started from the foreground process.
// TODO: add debounce?
//probably don't need this to be ISR for I2C macros.
static void execute_macro (uint8_t macro)
{
    if(!is_executing && state_get() == STATE_IDLE) {
        is_executing = true;
        command = plugin_settings.macro[macro].data;
        if(!(*command == '\0' || *command == 0xFF))     // If valid command
            protocol_enqueue_rt_command(run_macro);     // register run_macro function to be called from foreground process.
    }
}

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.

static const setting_group_detail_t macro_groups [] = {
    { Group_Root, Group_UserSettings, "Macros"}
};

// Write settings to non volatile storage (NVS).
static void keypad_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&jog, sizeof(jog_settings_t), true);
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(macro_settings_t), true);
}

static void keypad_settings_restore (void)
{
    jog.step_speed    = 100.0f;
    jog.slow_speed    = 600.0f;
    jog.fast_speed    = 3000.0f;
    jog.step_distance = 0.25f;
    jog.slow_distance = 500.0f;
    jog.fast_distance = 3000.0f;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&jog, sizeof(jog_settings_t), true);
    
    uint_fast8_t idx;

    // Register empty macro strings.
    for(idx = 0; idx < N_MACROS; idx++) {
        *plugin_settings.macro[idx].data = '\0';
    };

    //strcat(strcpy(command, "S"), ftoa(200, 0));
    //strcat(command, "M03");
    //plugin_settings.macro[4].data = command;
    strcpy(command, "S200M03");
    for(idx = 0; idx < N_MACROS; idx++) {
        plugin_settings.macro[4].data[idx] = command[idx];
    };

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(macro_settings_t), true);

}

static void keypad_settings_load (void)
{
    if((hal.nvs.memcpy_from_nvs((uint8_t *)&jog, nvs_address, sizeof(jog_settings_t), true) != NVS_TransferResult_OK) && 
       (hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(macro_settings_t), true) != NVS_TransferResult_OK))
        keypad_settings_restore();   
}

static setting_details_t setting_details = {
    .groups = macro_groups,
    .n_groups = sizeof(macro_groups) / sizeof(setting_group_detail_t),
    .settings = keypad_settings,
    .n_settings = sizeof(keypad_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = keypad_settings_descr,
    .n_descriptions = sizeof(keypad_settings_descr) / sizeof(setting_descr_t),
#endif
    .load = keypad_settings_load,
    .restore = keypad_settings_restore,
    .save = keypad_settings_save
};

// Returns 0 if no keycode enqueued
static char keypad_get_keycode (void)
{
    uint32_t data = 0, bptr = keybuf.tail;

    if(bptr != keybuf.head) {
        data = keybuf.buf[bptr++];               // Get next character, increment tmp pointer
        keybuf.tail = bptr & (KEYBUF_SIZE - 1);  // and update pointer
    }

    return data;
}

// BE WARNED: this function may be dangerous to use...
static char *strrepl (char *str, int c, char *str3)
{
    char tmp[30];
    char *s = strrchr(str, c);

    while(s) {
        strcpy(tmp, str3);
        strcat(tmp, s + 1);
        strcpy(s, tmp);
        s = strrchr(str, c);
    }

    return str;
}

static char *map_coord_system (coord_system_id_t id)
{
    uint8_t g5x = id + 54;

    strcpy(buf, uitoa((uint32_t)(g5x > 59 ? 59 : g5x)));
    if(g5x > 59) {
        strcat(buf, ".");
        strcat(buf, uitoa((uint32_t)(g5x - 59)));
    }

    return buf;
}

static void jog_command (char *cmd, char *to)
{
    strcat(strcpy(cmd, "$J=G91G21"), to);
}

static status_code_t disable_lock (sys_state_t state)
{
    status_code_t retval = Status_OK;

    if(state & (STATE_ALARM|STATE_ESTOP)) {

        control_signals_t control_signals = hal.control.get_state();

        // Block if self-test failed
        if(sys.alarm == Alarm_SelftestFailed)
            retval = Status_SelfTestFailed;
        // Block if e-stop is active.
        else if (control_signals.e_stop)
            retval = Status_EStop;
        // Block if safety door is ajar.
        else if (control_signals.safety_door_ajar)
            retval = Status_CheckDoor;
        // Block if safety reset is active.
        else if(control_signals.reset)
            retval = Status_Reset;
        else {
            grbl.report.feedback_message(Message_AlarmUnlock);
            state_set(STATE_IDLE);
        }
    } // Otherwise, no effect.

    return retval;
}

static void send_status_info (void)
{    
    int32_t current_position[N_AXIS]; // Copy current state of the system position variable
    float jog_modifier = 0;
    float print_position[N_AXIS];
    status_packet.a_coordinate = 0xffff;

    memcpy(current_position, sys.position, sizeof(sys.position));

    system_convert_array_steps_to_mpos(print_position, current_position);

    uint_fast8_t idx;
    float wco[N_AXIS];
    for (idx = 0; idx < N_AXIS; idx++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[idx] = gc_get_offset(idx);
        print_position[idx] -= wco[idx];
    }  
    
    status_packet.address = 0x01;
    
    switch(jogModify){
        case JogModify_1:
            jog_modifier = 1;
        break;
        case JogModify_01:
            jog_modifier = 0.1;                    
        break;
        case JogModify_001:
            jog_modifier = 0.01;                    
        break;  
    }
    
    switch (state_get()){
        case STATE_ALARM:
            status_packet.machine_state = 1;
            break;
        case STATE_ESTOP:
            status_packet.machine_state = 1;
            break;            
        case STATE_CYCLE:
            status_packet.machine_state = 2;
            break;
        case STATE_HOLD:
            status_packet.machine_state = 3;
            break;
        case STATE_TOOL_CHANGE:
            status_packet.machine_state = 4;
            break;
        case STATE_IDLE:
            status_packet.machine_state = 5;
            break;
        case STATE_HOMING:
            status_packet.machine_state = 6;
            break;   
        case STATE_JOG:
            status_packet.machine_state = 7;
            break;                                    
        default :
            status_packet.machine_state = 254;
            break;                                                        
    }
    status_packet.coolant_state = hal.coolant.get_state();
    status_packet.feed_override = sys.override.feed_rate;
    status_packet.spindle_override = sys.override.spindle_rpm;
    status_packet.spindle_stop = sys.override.spindle_stop.value;
    status_packet.spindle_rpm = sys.spindle_rpm;
    status_packet.alarm = (uint8_t) sys.alarm;
    status_packet.home_state = (uint8_t)(sys.homing.mask & sys.homed.mask);
    status_packet.jog_mode = (uint8_t) jogMode << 4 | (uint8_t) jogModify;
    status_packet.x_coordinate = print_position[0];
    status_packet.y_coordinate = print_position[1];
    status_packet.z_coordinate = print_position[2];
    #if N_AXIS > 3
    status_packet.a_coordinate = print_position[3];
    #else
    status_packet.a_coordinate = 0xFFFFFFFF;
    #endif

    status_packet.feed_rate = st_get_realtime_rate();

    switch(jogMode){
        case JogMode_Slow:
        status_packet.jog_stepsize = jog.slow_speed * jog_modifier;
        break;
        case JogMode_Fast:
        status_packet.jog_stepsize = jog.fast_speed * jog_modifier;
        break;
        default:
        status_packet.jog_stepsize = jog.step_distance * jog_modifier;
        break;
    }
    status_packet.current_wcs = gc_state.modal.coord_system.id;   
    
    I2C_Send (KEYPAD_I2CADDR, status_ptr, sizeof(Machine_status_packet), 0);
}

static void keypad_process_keypress (sys_state_t state)
{
    bool addedGcode, jogCommand = false;
    char command[35] = "", keycode = keypad_get_keycode();
    float jog_modifier = 0;

    spindle_state_t spindle_state;

    if(state == STATE_ESTOP)
        return;

    if(keycode) {

        if(keypad.on_keypress_preview && keypad.on_keypress_preview(keycode, state))
            return;

        switch(keycode) {

            case '?':                                    // pendant attach
                grbl.enqueue_realtime_command(CMD_STATUS_REPORT);
                send_status_info();
                break;
             case MACROUP:                                   //Macro 1 up
                //strcat(strcpy(command, "G10 L20 P0 Y"), ftoa(1.27, 5)); 
                execute_macro(0);            
                break;
             case MACRODOWN:                                   //Macro 3 down
                //strcat(strcpy(command, "G10 L20 P0 Y"), ftoa(-1.27, 5));
                execute_macro(2);              
                break;
             case MACROLEFT:                                   //Macro 2 right
                //strcat(strcpy(command, "G10 L20 P0 X"), ftoa(-1.27, 5));
                execute_macro(1); 
                break;
             case MACRORIGHT:                                   //Macro 4 left
                //strcat(strcpy(command, "G10 L20 P0 X"), ftoa(1.27, 5));
                execute_macro(3);             
                break;
             case SPINON:                                   //Macro 5 is special
                spindle_state = hal.spindle.get_state();
                if(!spindle_state.on){
                    //strcat(strcpy(command, "S"), ftoa(1500, 0));
                    //strcat(command, "M03");
                    execute_macro(4); 
                } else{
                    strcpy(command, "M05");
                }
                break;
             case MACROHOME:                                   // change WCS                
                if (gc_state.modal.coord_system.id  < N_WorkCoordinateSystems-1)
                    strcat(strcpy(command, "G"), map_coord_system(gc_state.modal.coord_system.id+1));    
                else
                    strcat(strcpy(command, "G"), map_coord_system(0x00));
                break;                
                break;
             case UNLOCK:  
                disable_lock(state_get());
                break;
             case RESET:                                   // Soft reset controller
                grbl.enqueue_realtime_command(CMD_RESET);
                break;
                                                                                                                                        
             case 'M':                                   // Mist override
                enqueue_accessory_override(CMD_OVERRIDE_COOLANT_MIST_TOGGLE);
                break;
            case 'C':                                   // Coolant override
                enqueue_accessory_override(CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE);
                break;

            case CMD_FEED_HOLD_LEGACY:                  // Feed hold
                grbl.enqueue_realtime_command(CMD_FEED_HOLD);
                break;

            case CMD_CYCLE_START_LEGACY:                // Cycle start
                grbl.enqueue_realtime_command(CMD_CYCLE_START);
                break;

            case CMD_MPG_MODE_TOGGLE:                   // Toggle MPG mode
                if(hal.driver_cap.mpg_mode)
                    stream_mpg_enable(hal.stream.type != StreamType_MPG);
                break;

            case '0':
            case '1':
            case '2':                                   // Set jog mode
                jogMode = (jogmode_t)(keycode - '0');
                break;

            case 'h':                                   // "toggle" jog mode
                jogMode = jogMode == JogMode_Step ? JogMode_Fast : (jogMode == JogMode_Fast ? JogMode_Slow : JogMode_Step);
                if(keypad.on_jogmode_changed)
                    keypad.on_jogmode_changed(jogMode);
                break;

            case 'm':                                   // cycle jog modifier
                jogModify = jogModify == JogModify_001 ? JogModify_1 : (jogModify == JogModify_1 ? JogModify_01 : JogModify_001);
                if(keypad.on_jogmodify_changed)
                    keypad.on_jogmodify_changed(jogModify);
                break;
            case 'H':                                   // Home axes
                strcpy(command, "$H");
                break;

         // Pass most of the top bit set commands trough unmodified

            case CMD_OVERRIDE_FEED_RESET:
            case CMD_OVERRIDE_FEED_COARSE_PLUS:
            case CMD_OVERRIDE_FEED_COARSE_MINUS:
            case CMD_OVERRIDE_FEED_FINE_PLUS:
            case CMD_OVERRIDE_FEED_FINE_MINUS:
            case CMD_OVERRIDE_RAPID_RESET:
            case CMD_OVERRIDE_RAPID_MEDIUM:
            case CMD_OVERRIDE_RAPID_LOW:
                enqueue_feed_override(keycode);
                send_status_info();
                break;

            case CMD_OVERRIDE_FAN0_TOGGLE:
            case CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE:
            case CMD_OVERRIDE_COOLANT_MIST_TOGGLE:
            case CMD_OVERRIDE_SPINDLE_RESET:
            case CMD_OVERRIDE_SPINDLE_COARSE_PLUS:
            case CMD_OVERRIDE_SPINDLE_COARSE_MINUS:
            case CMD_OVERRIDE_SPINDLE_FINE_PLUS:
            case CMD_OVERRIDE_SPINDLE_FINE_MINUS:
            case CMD_OVERRIDE_SPINDLE_STOP:
                enqueue_accessory_override(keycode);
                send_status_info();                
                break;

            case CMD_SAFETY_DOOR:
            case CMD_OPTIONAL_STOP_TOGGLE:
            case CMD_SINGLE_BLOCK_TOGGLE:
            case CMD_PROBE_CONNECTED_TOGGLE:
                grbl.enqueue_realtime_command(keycode);
                send_status_info();
                break;

         // Jogging

            case JOG_XR:                                // Jog X
                jog_command(command, "X?F");
                break;

            case JOG_XL:                                // Jog -X
                jog_command(command, "X-?F");
                break;

            case JOG_YF:                                // Jog Y
                jog_command(command, "Y?F");
                break;

            case JOG_YB:                                // Jog -Y
                jog_command(command, "Y-?F");
                break;

            case JOG_ZU:                                // Jog Z
                jog_command(command, "Z?F");
                break;

            case JOG_ZD:                                // Jog -Z
                jog_command(command, "Z-?F");
                break;

            case JOG_XRYF:                              // Jog XY
                jog_command(command, "X?Y?F");
                break;

            case JOG_XRYB:                              // Jog X-Y
                jog_command(command, "X?Y-?F");
                break;

            case JOG_XLYF:                              // Jog -XY
                jog_command(command, "X-?Y?F");
                break;

            case JOG_XLYB:                              // Jog -X-Y
                jog_command(command, "X-?Y-?F");
                break;

            case JOG_XRZU:                              // Jog XZ
                jog_command(command, "X?Z?F");
                break;

            case JOG_XRZD:                              // Jog X-Z
                jog_command(command, "X?Z-?F");
                break;

            case JOG_XLZU:                              // Jog -XZ
                jog_command(command, "X-?Z?F");
                break;

            case JOG_XLZD:                              // Jog -X-Z
                jog_command(command, "X-?Z-?F");
                break;

             case MACRORAISE:                           //  Jog +A
                jog_command(command, "A?F");
                break;

             case MACROLOWER:                           // Jog -A
                jog_command(command, "A-?F");
                break;                
        }

        if(command[0] != '\0') {
            // add distance and speed to jog commands
            switch(jogModify){
                case JogModify_1:
                    jog_modifier = 1;
                break;
                case JogModify_01:
                    jog_modifier = 0.1;                    
                break;
                case JogModify_001:
                    jog_modifier = 0.01;                    
                break;                                                            
            }
            if((jogCommand = (command[0] == '$' && command[1] == 'J'))) switch(jogMode) {
                case JogMode_Slow:
                    strrepl(command, '?', ftoa(jog.slow_distance, 0));
                    strcat(command, ftoa(jog.slow_speed*jog_modifier, 0));
                    break;

                case JogMode_Step:
                    strrepl(command, '?', ftoa(jog.step_distance*jog_modifier, gc_state.modal.units_imperial ? 4 : 3));
                    strcat(command, ftoa(jog.step_speed, 0));
                    break;

                default:
                    strrepl(command, '?', ftoa(jog.fast_distance, 0));
                    strcat(command, ftoa(jog.fast_speed*jog_modifier, 0));
                    break;
            }
            if(!(jogCommand && keyreleased)) { // key still pressed? - do not execute jog command if released!             
                addedGcode = grbl.enqueue_gcode((char *)command);
                jogging = jogging || (jogCommand && addedGcode);
            }
        }
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt){
        hal.stream.write("[PLUGIN:KEYPAD v1.3 INTERTEST]"  ASCII_EOL);
        hal.stream.write("[PLUGIN:Macro plugin v0.02]" ASCII_EOL);
    }
}

ISR_CODE bool ISR_FUNC(keypad_enqueue_keycode)(char c)
{
    uint32_t bptr = (keybuf.head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

#if MPG_MODE != 2
    if(c == CMD_MPG_MODE_TOGGLE)
        return true;
#endif

    if(c == CMD_JOG_CANCEL || c == ASCII_CAN) {
        keyreleased = true;
        if(jogging) {
            jogging = false;
            grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
        }
        keybuf.tail = keybuf.head;      // Flush keycode buffer.
    } else if(bptr != keybuf.tail) {    // If not buffer full
        keybuf.buf[keybuf.head] = c;    // add data to buffer
        keybuf.head = bptr;             // and update pointer.
        keyreleased = false;
        // Tell foreground process to process keycode
        if(nvs_address != 0)
            protocol_enqueue_rt_command(keypad_process_keypress);
    }

    return true;
}

#if KEYPAD_ENABLE == 1

ISR_CODE static void ISR_FUNC(i2c_enqueue_keycode)(char c)
{
    uint32_t bptr = (keybuf.head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

    if(bptr != keybuf.tail) {           // If not buffer full
        keybuf.buf[keybuf.head] = c;    // add data to buffer
        keybuf.head = bptr;             // and update pointer
        // Tell foreground process to process keycode
        if(nvs_address != 0)
            protocol_enqueue_rt_command(keypad_process_keypress);
    }
}

ISR_CODE bool ISR_FUNC(keypad_strobe_handler)(uint_fast8_t id, bool keydown)
{
    keyreleased = !keydown;

    if(keydown)
        I2C_GetKeycode(KEYPAD_I2CADDR, i2c_enqueue_keycode);

    else if(jogging) {
        jogging = false;
        grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
        keybuf.tail = keybuf.head; // flush keycode buffer
    }
    else {
        keybuf.tail = keybuf.head; // flush keycode buffer
    }

    return true;
}

static void onStateChanged (sys_state_t state)
{
    send_status_info();
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    //send_status_info();
}

static void keypad_poll (void)
{
    static uint32_t last_ms;

    uint32_t ms = hal.get_elapsed_ticks();

    if (state_get() == STATE_JOG && ms > last_ms + (50)){ //check faster during jogging.
        send_status_info();
        last_ms = ms;}
    else if(ms < last_ms + SEND_STATUS_DELAY){ // check once every update period
        return;}
    send_status_info();
    last_ms = ms;
}

static void keypad_poll_realtime (sys_state_t grbl_state)
{
    on_execute_realtime(grbl_state);

    keypad_poll();
}

static void keypad_poll_delay (sys_state_t grbl_state)
{
    on_execute_delay(grbl_state);

    keypad_poll();
}

static void jogmode_changed (jogmode_t jogMode)
{
    send_status_info();
}

static void jogmodify_changed (jogmodify_t jogModify)
{
    send_status_info();
}

bool keypad_init (void)
{
    if(hal.irq_claim(IRQ_I2C_Strobe, 0, keypad_strobe_handler) && 
      (nvs_address = nvs_alloc(sizeof(jog_settings_t))) && 
      (nvs_address = nvs_alloc(sizeof(macro_settings_t)))) {

        // Hook into the driver reset chain so we
        // can restore normal operation if a reset happens
        // when a macro is running.
        driver_reset = hal.driver_reset;
        hal.driver_reset = plugin_reset;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = keypad_poll_realtime;

        on_execute_delay = grbl.on_execute_delay;
        grbl.on_execute_delay = keypad_poll_delay;

        settings_register(&setting_details);       

        on_jogmode_changed = keypad.on_jogmode_changed;
        keypad.on_jogmode_changed = jogmode_changed;

        on_jogmodify_changed = keypad.on_jogmodify_changed;
        keypad.on_jogmodify_changed = jogmodify_changed;

        if(keypad.on_jogmode_changed)
            keypad.on_jogmode_changed(jogMode);

        if(keypad.on_jogmodify_changed)
            keypad.on_jogmodify_changed(jogModify);                               

        on_state_change = grbl.on_state_change;             // Subscribe to the state changed event by saving away the original
        grbl.on_state_change = onStateChanged;              // function pointer and adding ours to the chain.

        on_realtime_report = grbl.on_realtime_report;       // Subscribe to realtime report events AKA ? reports
        grbl.on_realtime_report = onRealtimeReport;         // Nothing here yet
     
    }   

    return nvs_address != 0;
}

#else

bool keypad_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(jog_settings_t)))) {

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        settings_register(&setting_details);

        if(keypad.on_jogmode_changed)
            keypad.on_jogmode_changed(jogMode);
    }

    return nvs_address != 0;
}

#endif

#endif
