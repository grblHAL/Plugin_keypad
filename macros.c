/*
  macros.c - plugin for binding macros to aux input pins and/or key codes

  Part of grblHAL keypad plugins

  Copyright (c) 2021-2024 Terje Io

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

/*
  Up to 8 macros can be bound to input pins and/or keypad keypresses by changing the N_MACROS symbol below.
  Each macro can be up to 127 characters long, blocks (lines) are separated by a vertical bar character: |
  Setting numbers $490 - $497 are for defining the macro content.
  Setting numbers $500 - $507 are for configuring which aux input port to assign to each macro.
  Setting numbers $590 - $599 are for binding an action to the aux port.
  NOTES: If the driver does not support mapping of port numbers settings $500 - $505 will not be available.
         The mapped pins has to be interrupt capable and support falling interrupt mode.
         The controller must be in Idle mode when starting macros.

  Examples:
    $450=G0Y5|G1X0F50
    $451=G0x0Y0Z0

  Tip: use the $pins command to check the port mapping.
*/

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if MACROS_ENABLE && MACROS_ENABLE <= 3

#include <stdio.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"

#include "macros.h"

#ifndef N_MACROS
#if MACROS_ENABLE == 2
#define N_MACROS 4 // MAX 8
#else
#define N_MACROS 2 // MAX 8
#endif
#endif

#ifndef MACRO_LENGTH_MAX
#if N_MACROS > 6
#define MACRO_LENGTH_MAX 63
#elif N_MACROS > 4
#define MACRO_LENGTH_MAX 83
#else
#define MACRO_LENGTH_MAX 127
#endif
#endif

// Sanity check
#if N_MACROS > 8
#undef N_MACROS
#define N_MACROS 8
#endif

#define MACRO_OPTS { .subgroups = Off, .increment = 1 }
#define BUTTON_ACTIONS "Macro,Cycle start,Feed hold,Park,Reset,Spindle stop (during feed hold),Mist toggle,Flood toggle,Probe connected toggle,Optional stop toggle,Single block mode toggle"

static uint8_t action[] = {
    0, // run macro
    CMD_CYCLE_START,
    CMD_FEED_HOLD,
    CMD_SAFETY_DOOR,
    CMD_RESET,
    CMD_OVERRIDE_SPINDLE_STOP,
    CMD_OVERRIDE_COOLANT_MIST_TOGGLE,
    CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE,
    CMD_PROBE_CONNECTED_TOGGLE,
    CMD_OPTIONAL_STOP_TOGGLE,
    CMD_SINGLE_BLOCK_TOGGLE
};

#if MACROS_ENABLE & 0x02

#if !KEYPAD_ENABLE
#error "Macro plugin requires the keypad plugin enabled!"
#endif

#include "keypad.h"

#ifndef MACRO_KEY0
#define MACRO_KEY0 0x18
#endif
#ifndef MACRO_KEY1
#define MACRO_KEY1 0x19
#endif
#ifndef MACRO_KEY2
#define MACRO_KEY2 0x1B
#endif
#ifndef MACRO_KEY3
#define MACRO_KEY3 0x1A
#endif
#ifndef MACRO_KEY4
#define MACRO_KEY4 0x7D
#endif
#ifndef MACRO_KEY5
#define MACRO_KEY5 0x7C
#endif
#ifndef MACRO_KEY6
#define MACRO_KEY6 0x7E
#endif
#ifndef MACRO_KEY7
#define MACRO_KEY7 0x7F
#endif

static on_keypress_preview_ptr on_keypress_preview;

#endif

typedef struct {
    uint8_t port;
    uint8_t action_idx;
    char data[MACRO_LENGTH_MAX + 1];
} macro_setting_t;

typedef struct {
    macro_setting_t macro[N_MACROS];
} macro_settings_t;

static bool can_map_ports = false;
static uint8_t n_ports = 0;
static uint8_t port[N_MACROS];
static char max_port[4], *command, format[8], max_length[5];
static macro_id_t macro_id = 0;
static nvs_address_t nvs_address;
static macro_settings_t plugin_settings;
static on_report_options_ptr on_report_options;
static on_macro_execute_ptr on_macro_execute;
static on_macro_return_ptr on_macro_return = NULL;
static status_message_ptr status_message = NULL;
static driver_reset_ptr driver_reset;
static io_stream_t active_stream;

static int16_t get_macro_char (void);
static status_code_t trap_status_messages (status_code_t status_code);

// Ends macro execution if currently running
// and restores normal operation.
static void end_macro (void)
{
    if(hal.stream.read == get_macro_char)
        memcpy(&hal.stream, &active_stream, sizeof(io_stream_t));

    if(macro_id) {

        macro_id = 0;

        grbl.on_macro_return = on_macro_return;
        on_macro_return = NULL;

        if(grbl.report.status_message == trap_status_messages)
            grbl.report.status_message = status_message;

        status_message = NULL;
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

    if(*command == '\0') {          // End of macro?
        if(eol_ok) {
            end_macro();            // Done
            return SERIAL_NO_DATA;  // ...
        }
        eol_ok = true;

        return ASCII_LF;  // Return a linefeed if the last character was not a linefeed.
    }

    char c = *command++;    // Get next character.

    if((eol_ok = c == '|')) // If character is vertical bar
        c = ASCII_LF;       // return a linefeed character.

    return (uint16_t)c;
}

// If an error is detected macro execution will be stopped and the status_code reported.
static status_code_t trap_status_messages (status_code_t status_code)
{
    if(hal.stream.read != get_macro_char)
        status_code = status_message(status_code);

    else if(status_code != Status_OK) {

        char msg[30];
        sprintf(msg, "error %d in macro", (uint8_t)status_code);
        report_message(msg, Message_Warning);

        if(grbl.report.status_message == trap_status_messages && (grbl.report.status_message = status_message))
            status_code = grbl.report.status_message(status_code);

        end_macro();
    }

    return status_code;
}

// Actual start of macro execution.
static void run_macro (void *data)
{
    if(state_get() == STATE_IDLE && hal.stream.read != get_macro_char) {

        memcpy(&active_stream, &hal.stream, sizeof(io_stream_t));   // Redirect input stream to read from the macro instead
        hal.stream.read = get_macro_char;                           // the active stream. This ensures that input streams are not mingled.
        hal.stream.file = NULL;                                     // Input stream is not file based.

        status_message = grbl.report.status_message;        // Add trap for status messages
        grbl.report.status_message = trap_status_messages;  // so we can terminate on errors.

        on_macro_return = grbl.on_macro_return;
        grbl.on_macro_return = end_macro;
    } else
        macro_id = 0;
}

static status_code_t macro_execute (macro_id_t macro)
{
    bool ok = false;

    if(macro_id == 0 && macro > 0 && macro <= N_MACROS && state_get() == STATE_IDLE) {
        command = plugin_settings.macro[macro - 1].data;
        if((ok = !(*command == '\0' || *command == 0xFF))) {
            macro_id = macro;
            run_macro(NULL);
        }
    }

    return ok ? Status_OK : (on_macro_execute ? on_macro_execute(macro) : Status_Unhandled);
}

#if MACROS_ENABLE & 0x01

// On falling interrupt run macro if machine is in Idle state.
// Since this function runs in an interrupt context actual start of execution
// is registered as a single run task to be started from the foreground process.
// TODO: add debounce?
ISR_CODE static void ISR_FUNC(execute_macro)(uint8_t irq_port, bool is_high)
{
    if(!is_high && macro_id == 0) {

        // Determine macro to run from port number
        uint_fast8_t idx = N_MACROS;
        do {
            idx--;
        } while(idx && port[idx] != irq_port);

        if(plugin_settings.macro[idx].action_idx)
            grbl.enqueue_realtime_command(action[plugin_settings.macro[idx].action_idx]);
        else if(state_get() == STATE_IDLE) {
            macro_id = idx + 1;
            command = plugin_settings.macro[idx].data;
            if(!(*command == '\0' || *command == 0xFF))             // If valid command
                protocol_enqueue_foreground_task(run_macro, NULL);  // register run_macro function to be called from foreground process.
        }
    }
}

#endif

#if MACROS_ENABLE & 0x02

static bool keypress_preview (const char c, uint_fast16_t state)
{
    int32_t macro = -1;

    switch(c) {

        case MACRO_KEY0:
            macro = 0;
            break;
#if N_MACROS > 1
        case MACRO_KEY1:
            macro = 1;
            break;
#endif
#if N_MACROS > 2
        case MACRO_KEY2:
            macro = 2;
            break;
#endif
#if N_MACROS > 3
        case MACRO_KEY3:
            macro = 3;
            break;
#endif
#if N_MACROS > 4
        case MACRO_KEY4:
            macro = 4;
            break;
#endif
#if N_MACROS > 5
        case MACRO_KEY5:
            macro = 5;
            break;
#endif
#if N_MACROS > 6
        case MACRO_KEY6:
            macro = 6;
            break;
#endif
#if N_MACROS > 7
        case MACRO_KEY7:
            macro = 7;
            break;
#endif
    }

    if(macro != -1) {
        command = plugin_settings.macro[macro].data;
        if(!(*command == '\0' || *command == 0xFF)) // If valid command
            run_macro(NULL);                        // run macro.
    }

    return macro != -1 || (on_keypress_preview && on_keypress_preview(c, state));
}

#endif

// Add info about our settings for $help and enumerations.
// Potentially used by senders for settings UI.

static const setting_group_detail_t macro_groups [] = {
    { Group_Root, Group_UserSettings, "Macros"}
};

static setting_id_t normalize_id (setting_id_t setting, uint_fast16_t *idx)
{
    *idx = setting % 10;

    return (setting_id_t)(setting - *idx);
}

static status_code_t macro_set (setting_id_t setting, char *value)
{
    uint_fast16_t idx;

    normalize_id(setting, &idx);

    if(idx < N_MACROS)
        strcpy(plugin_settings.macro[idx].data, value);

    return idx < N_MACROS ? Status_OK : Status_Unhandled;
}

static char *macro_get (setting_id_t setting)
{
    uint_fast16_t idx;

    normalize_id(setting, &idx);

    return idx < N_MACROS ? plugin_settings.macro[idx].data : NULL;
}

#if MACROS_ENABLE & 0x01

static status_code_t macro_set_int (setting_id_t setting, uint_fast16_t value)
{
    uint_fast16_t idx;
    status_code_t status = Status_OK;

    setting = normalize_id(setting, &idx);

    if(idx < N_MACROS) switch(setting) {

        case Setting_MacroPortBase:
            plugin_settings.macro[idx].port = value;
            break;

        case Setting_ButtonActionBase:
            plugin_settings.macro[idx].action_idx = value;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static uint_fast16_t macro_get_int (setting_id_t setting)
{
    uint_fast16_t idx, value = 0;

    setting = normalize_id(setting, &idx);

    if(idx < N_MACROS) switch(setting) {

        case Setting_MacroPortBase:
            value = plugin_settings.macro[idx].port;
            break;

        case Setting_ButtonActionBase:
            value = plugin_settings.macro[idx].action_idx;
            break;

        default:
            break;
    }

    return value;
}

#endif

static bool is_setting_available (const setting_detail_t *setting)
{
#if MACROS_ENABLE & 0x01

    uint_fast16_t idx;

    normalize_id(setting->id, &idx);

    return can_map_ports && idx < N_MACROS;
#else
    return true;
#endif
}

static const setting_detail_t macro_settings[] = {
    { Setting_MacroBase, Group_UserSettings, "Macro ?", NULL, Format_String, format, "0", max_length, Setting_NonCoreFn, macro_set, macro_get, NULL, MACRO_OPTS },
#if MACROS_ENABLE & 0x01
    { Setting_MacroPortBase, Group_AuxPorts, "Macro ? port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCoreFn, macro_set_int, macro_get_int, is_setting_available, MACRO_OPTS },
    { Setting_ButtonActionBase, Group_UserSettings, "Button ? action", NULL, Format_RadioButtons, BUTTON_ACTIONS, NULL, NULL, Setting_NonCoreFn, macro_set_int, macro_get_int, NULL, MACRO_OPTS },
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t macro_settings_descr[] = {
    { Setting_MacroBase, "Macro content, separate blocks (lines) with the vertical bar character |." },
#if MACROS_ENABLE & 0x01
    { Setting_MacroPortBase, "Aux port number to use for the trigger pin input." SETTINGS_HARD_RESET_REQUIRED },
    { Setting_ButtonActionBase, "Action to take when the pin is triggered."  },
#endif
};

#endif

// Write settings to non volatile storage (NVS).
static void macro_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(macro_settings_t), true);
}

// Restore default settings and write to non volatile storage (NVS).
static void macro_settings_restore (void)
{
    uint_fast8_t idx = N_MACROS, port = n_ports > N_MACROS ? n_ports - N_MACROS : 0;

    memset(&plugin_settings, 0xFF, sizeof(macro_settings_t));

    // Register empty macro strings and set default port numbers if mapping is available.
    for(idx = 0; idx < N_MACROS; idx++) {

        plugin_settings.macro[idx].action_idx = 0;
        *plugin_settings.macro[idx].data = '\0';

        switch(idx) {
#ifdef MACRO_1_AUXIN
            case 0:
                plugin_settings.macro[idx].port = MACRO_1_AUXIN;
  #ifdef MACRO_1_BUTTONACTION
                plugin_settings.macro[idx].action_idx = MACRO_1_BUTTONACTION;
  #endif
                break;
#endif
#ifdef MACRO_2_AUXIN
            case 1:
                plugin_settings.macro[idx].port = MACRO_2_AUXIN;
  #ifdef MACRO_2_BUTTONACTION
              plugin_settings.macro[idx].action_idx = MACRO_2_BUTTONACTION;
  #endif
                break;
#endif
#ifdef MACRO_3_AUXIN
            case 2:
                plugin_settings.macro[idx].port = MACRO_3_AUXIN;
  #ifdef MACRO_3_BUTTONACTION
              plugin_settings.macro[idx].action_idx = MACRO_3_BUTTONACTION;
  #endif
                break;
#endif
#ifdef MACRO_4_AUXIN
            case 3:
                plugin_settings.macro[idx].port = MACRO_4_AUXIN;
  #ifdef MACRO_4_BUTTONACTION
              plugin_settings.macro[idx].action_idx = MACRO_4_BUTTONACTION;
  #endif
                break;
#endif
            default:
                plugin_settings.macro[idx].port = can_map_ports ? port++ : 0xFF;
                break;
        }
    };

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(macro_settings_t), true);
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
static void macro_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(macro_settings_t), true) != NVS_TransferResult_OK)
        macro_settings_restore();

#if MACROS_ENABLE & 0x01

    // If port mapping is available try to claim ports as configured.

    uint_fast8_t idx = N_MACROS, n_ok = 0;

    if(can_map_ports) {

        xbar_t *pin = NULL;

        do {
            idx--;
            port[idx] = plugin_settings.macro[idx].port;
            if(hal.port.get_pin_info)
                pin = hal.port.get_pin_info(Port_Digital, Port_Input, port[idx]);
            if(pin && !(pin->cap.irq_mode & IRQ_Mode_Falling))                          // Is port interrupt capable?
                port[idx] = 0xFF;                                                       // No, flag it as not claimed.
            else if(ioport_claim(Port_Digital, Port_Input, &port[idx], "Macro pin")) {  // Try to claim the port.
                if(pin->cap.debounce) {
                    gpio_in_config_t config = {
                        .debounce = On,
                        .pull_mode = PullMode_Up
                    };
                    pin->config(pin, &config, false);
                }
            } else
                port[idx] = 0xFF;                                                       // If not successful flag it as not claimed.

        } while(idx);
    }

    // Then try to register the interrupt handler.
    idx = N_MACROS;
    do {
        idx--;
        if(port[idx] != 0xFF && hal.port.register_interrupt_handler(port[idx], IRQ_Mode_Falling, execute_macro))
            n_ok++;
    } while(idx);

    if(n_ok < N_MACROS)
        protocol_enqueue_foreground_task(report_warning, "Macro plugin failed to claim all needed ports!");
#endif
}

static bool macro_settings_iterator (const setting_detail_t *setting, setting_output_ptr callback, void *data)
{
    uint_fast16_t idx;

    for(idx = 0; idx < N_MACROS; idx++)
        callback(setting, idx, data);

    return true;
}

// Settings descriptor used by the core when interacting with this plugin.
static setting_details_t setting_details = {
    .groups = macro_groups,
    .n_groups = sizeof(macro_groups) / sizeof(setting_group_detail_t),
    .settings = macro_settings,
    .n_settings = sizeof(macro_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = macro_settings_descr,
    .n_descriptions = sizeof(macro_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = macro_settings_save,
    .load = macro_settings_load,
    .restore = macro_settings_restore,
    .iterator = macro_settings_iterator
};

// Add info about our plugin to the $I report.
static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Macro plugin", "0.08");
}

void macros_init (void)
{
#if MACROS_ENABLE & 0x01
    bool ok = (n_ports = ioports_available(Port_Digital, Port_Input)) > N_MACROS;

    if(ok && !(can_map_ports = ioport_can_claim_explicit())) {

        // Driver does not support explicit pin claiming, claim the highest numbered ports instead.

        uint_fast8_t idx = N_MACROS;

        do {
            idx--;
            if(!(ok = ioport_claim(Port_Digital, Port_Input, &port[idx], "Macro pin")))
                port[idx] = 0xFF;
        } while(idx);
    }
#else
    bool ok = true;
#endif

    // If enough free non volatile memory register our plugin with the core.
    if(ok && (nvs_address = nvs_alloc(sizeof(macro_settings_t)))) {

        // Register settings.
        settings_register(&setting_details);

        // Used for setting value validation.
        strcpy(max_port, uitoa(n_ports ? n_ports - 1 : 0));

#if MACROS_ENABLE & 0x02
        on_keypress_preview = keypad.on_keypress_preview;
        keypad.on_keypress_preview = keypress_preview;
#endif

        // Add our plugin to the $I options report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        on_macro_execute = grbl.on_macro_execute;
        grbl.on_macro_execute = macro_execute;

        // Hook into the driver reset chain so we
        // can restore normal operation if a reset happens
        // when a macro is running.
        driver_reset = hal.driver_reset;
        hal.driver_reset = plugin_reset;

        strcpy(max_length, uitoa(MACRO_LENGTH_MAX));
        strcat(strcat(strcpy(format, "x("), max_length), ")");

    } else
        protocol_enqueue_foreground_task(report_warning, "Macro plugin failed to initialize!");
}

#endif // MACROS_ENABLE
