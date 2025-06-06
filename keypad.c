/*
  keypad.c - I2C/UART keypad plugin

  Part of grblHAL keypad plugins

  Copyright (c) 2017-2025 Terje Io

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

#include "driver.h"

#if KEYPAD_ENABLE > 0 && KEYPAD_ENABLE <= 2

#include <string.h>

#include "keypad.h"

#include "grbl/plugins.h"
#include "grbl/report.h"
#include "grbl/override.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/state_machine.h"

#define KEYPAD_VERSION "1.42"

typedef struct {
    char buf[KEYBUF_SIZE];
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
} keybuffer_t;

static bool jogging = false, keyreleased = true;
static jog_settings_t jog;
static jogdata_t jogdata = {
    .modifier[0] = 1.0f,
    .modifier[1] = 0.1f,
    .modifier[2] = 0.01f,
    .modifier_index = 0,
    .mode = JogMode_Fast
};
static keybuffer_t keybuf = {0};
static uint32_t nvs_address;
static on_report_options_ptr on_report_options;

keypad_t keypad = {0};

static const setting_detail_t keypad_settings[] = {
    { Setting_JogStepSpeed, Group_Jogging, "Step jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.step_speed, NULL, NULL },
    { Setting_JogSlowSpeed, Group_Jogging, "Slow jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.slow_speed, NULL, NULL },
    { Setting_JogFastSpeed, Group_Jogging, "Fast jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.fast_speed, NULL, NULL },
    { Setting_JogStepDistance, Group_Jogging, "Step jog distance", "mm", Format_Decimal, "#0.000", NULL, NULL, Setting_NonCore, &jog.step_distance, NULL, NULL },
    { Setting_JogSlowDistance, Group_Jogging, "Slow jog distance", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.slow_distance, NULL, NULL },
    { Setting_JogFastDistance, Group_Jogging, "Fast jog distance", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &jog.fast_distance, NULL, NULL }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t keypad_settings_descr[] = {
    { Setting_JogStepSpeed, "Step jogging speed in millimeters per minute." },
    { Setting_JogSlowSpeed, "Slow jogging speed in millimeters per minute." },
    { Setting_JogFastSpeed, "Fast jogging speed in millimeters per minute." },
    { Setting_JogStepDistance, "Jog distance for single step jogging." },
    { Setting_JogSlowDistance, "Jog distance before automatic stop." },
    { Setting_JogFastDistance, "Jog distance before automatic stop." }
};

#endif

static void keypad_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&jog, sizeof(jog_settings_t), true);
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
}

static void keypad_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&jog, nvs_address, sizeof(jog_settings_t), true) != NVS_TransferResult_OK)
        keypad_settings_restore();

    memcpy(&jogdata.settings, &jog, sizeof(jog_settings_t));

    if(keypad.on_jogdata_changed)
        keypad.on_jogdata_changed(&jogdata);
}

static setting_details_t setting_details = {
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

static void jog_command (char *cmd, char *to)
{
    strcat(strcpy(cmd, "$J=G91G21"), to);
}

static void keypad_process_keypress (void *data)
{
    bool addedGcode, jogCommand = false;
    char command[35] = "", keycode = keypad_get_keycode();
    sys_state_t state = state_get();

    if((state & (STATE_ESTOP|STATE_ALARM)) &&
        !(keycode == CMD_STATUS_REPORT ||
           keycode == CMD_STATUS_REPORT_LEGACY ||
            keycode == CMD_RESET ||
             keycode == CMD_MPG_MODE_TOGGLE ||
              keycode == 'X' || keycode == 'H'))
        return;

    if(keycode) {

        if(keypad.on_keypress_preview && keypad.on_keypress_preview(keycode, state))
            return;

        switch(keycode) {

            case 'M':                                   // Mist override
                enqueue_coolant_override(CMD_OVERRIDE_COOLANT_MIST_TOGGLE);
                break;

            case 'C':                                   // Coolant override
                enqueue_coolant_override(CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE);
                break;

            case CMD_FEED_HOLD:                         // Feed hold
            case CMD_FEED_HOLD_LEGACY:
                grbl.enqueue_realtime_command(CMD_FEED_HOLD);
                break;

            case CMD_CYCLE_START:                       // Cycle start
                if(grbl.enqueue_realtime_command(CMD_CYCLE_START))
                    sys.report.cycle_start = settings.status_report.pin_state;
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
                jogdata.mode = (jogmode_t)(keycode - '0');
                if(keypad.on_jogmode_changed)
                    keypad.on_jogmode_changed(jogdata.mode);
                if(keypad.on_jogdata_changed)
                    keypad.on_jogdata_changed(&jogdata);
                break;

            case 'h':                                   // Cycle jog mode
                jogdata.mode = jogdata.mode == JogMode_Step ? JogMode_Fast : (jogdata.mode == JogMode_Fast ? JogMode_Slow : JogMode_Step);
                if(keypad.on_jogmode_changed)
                    keypad.on_jogmode_changed(jogdata.mode);
                if(keypad.on_jogdata_changed)
                    keypad.on_jogdata_changed(&jogdata);
                break;

            case 'm':                                   // Cycle jog modifier
                if(++jogdata.modifier_index >= sizeof(jogdata.modifier) / sizeof(float))
                    jogdata.modifier_index = 0;
                if(keypad.on_jogdata_changed)
                    keypad.on_jogdata_changed(&jogdata);
                break;

            case 'o':                                   // Cycle coordinate system
                strcpy(command, gc_coord_system_to_str(gc_state.modal.coord_system.id < N_WorkCoordinateSystems - 1 ? gc_state.modal.coord_system.id + 1 : 0));
                break;

            case 'H':                                   // Home axes
                strcpy(command, "$H");
                break;

            case 'X':                                   // Unlock
                strcpy(command, "$X");
                break;

         // Feed rate and spindle overrides

            case 'I':                                   // Feed rate coarse override -10%
                enqueue_feed_override(CMD_OVERRIDE_FEED_RESET);
                break;

            case 'i':                                   // Feed rate coarse override +10%
                enqueue_feed_override(CMD_OVERRIDE_FEED_COARSE_PLUS);
                break;

            case 'j':                                   // Feed rate fine override +1%
                enqueue_feed_override(CMD_OVERRIDE_FEED_COARSE_MINUS);
                break;

            case 'K':                                  // Spindle RPM coarse override -10%
                enqueue_spindle_override(CMD_OVERRIDE_SPINDLE_RESET);
                break;

            case 'k':                                   // Spindle RPM coarse override +10%
                enqueue_spindle_override(CMD_OVERRIDE_SPINDLE_COARSE_PLUS);
                break;

            case 'z':                                   // Spindle RPM fine override +1%
                enqueue_spindle_override(CMD_OVERRIDE_SPINDLE_COARSE_MINUS);
                break;

         // Pass most of the top bit set commands through unmodified

            case CMD_OVERRIDE_FEED_RESET:
            case CMD_OVERRIDE_FEED_COARSE_PLUS:
            case CMD_OVERRIDE_FEED_COARSE_MINUS:
            case CMD_OVERRIDE_FEED_FINE_PLUS:
            case CMD_OVERRIDE_FEED_FINE_MINUS:
            case CMD_OVERRIDE_RAPID_RESET:
            case CMD_OVERRIDE_RAPID_MEDIUM:
            case CMD_OVERRIDE_RAPID_LOW:
                enqueue_feed_override(keycode);
                break;

            case CMD_OVERRIDE_FAN0_TOGGLE:
            case CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE:
            case CMD_OVERRIDE_COOLANT_MIST_TOGGLE:
                enqueue_coolant_override(keycode);
                break;

            case CMD_OVERRIDE_SPINDLE_RESET:
            case CMD_OVERRIDE_SPINDLE_COARSE_PLUS:
            case CMD_OVERRIDE_SPINDLE_COARSE_MINUS:
            case CMD_OVERRIDE_SPINDLE_FINE_PLUS:
            case CMD_OVERRIDE_SPINDLE_FINE_MINUS:
            case CMD_OVERRIDE_SPINDLE_STOP:
                enqueue_spindle_override(keycode);
                break;

            case CMD_RESET:
            case CMD_SAFETY_DOOR:
            case CMD_STATUS_REPORT:
            case CMD_STATUS_REPORT_LEGACY:
            case CMD_OPTIONAL_STOP_TOGGLE:
            case CMD_SINGLE_BLOCK_TOGGLE:
            case CMD_PROBE_CONNECTED_TOGGLE:
                grbl.enqueue_realtime_command(keycode);
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
#if N_AXIS > 3
             case JOG_AR:                               //  Jog +A
                jog_command(command, "A?F");
                break;

             case JOG_AL:                               // Jog -A
                jog_command(command, "A-?F");
                break;
#endif
        }

       if(command[0] == '$') {

            // add distance and speed to jog commands
            if((jogCommand = command[1] == 'J')) {

                float modifier = jogdata.modifier[jogdata.modifier_index];

                switch(jogdata.mode) {

                    case JogMode_Slow:
                        strrepl(command, '?', ftoa(jog.slow_distance, 0));
                        strcat(command, ftoa(jog.slow_speed * modifier, 0));
                        break;

                    case JogMode_Step:
                        strrepl(command, '?', ftoa(jog.step_distance * modifier, gc_state.modal.units_imperial ? 4 : 3));
                        strcat(command, ftoa(jog.step_speed, 0));
                        break;

                    default:
                        strrepl(command, '?', ftoa(jog.fast_distance, 0));
                        strcat(command, ftoa(jog.fast_speed * modifier, 0));
                        break;
                }
            } else if(command[1] == 'X' || command[1] == 'H') {
                system_execute_line(command);
                return;
            }
        }

        if(command[0] && !(jogCommand && keyreleased)) { // key still pressed? - do not execute jog command if released!
            addedGcode = grbl.enqueue_gcode((char *)command);
            jogging = jogging || (jogCommand && addedGcode);
        }
    }
}

#if KEYPAD_ENABLE == 1

static bool connected;

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Keypad", connected ? KEYPAD_VERSION : KEYPAD_VERSION " (not connected)");
}

ISR_CODE static void ISR_FUNC(i2c_enqueue_keycode)(char c)
{
    uint32_t bptr = (keybuf.head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

    if(bptr != keybuf.tail) {           // If not buffer full
        keybuf.buf[keybuf.head] = c;    // add data to buffer
        keybuf.head = bptr;             // and update pointer
        // Tell foreground process to process keycode
        if(nvs_address != 0)
            task_add_immediate(keypad_process_keypress, NULL);
    }
}

static void i2c_get_key (void *data)
{
    i2c_get_keycode(KEYPAD_I2CADDR, i2c_enqueue_keycode);
}

ISR_CODE bool ISR_FUNC(keypad_strobe_handler)(uint_fast8_t id, bool keydown)
{
    keyreleased = !keydown;

    if(keydown)
        task_add_immediate(i2c_get_key, NULL);
    else if(jogging) {
        jogging = false;
        grbl.enqueue_realtime_command(CMD_JOG_CANCEL);
        keybuf.tail = keybuf.head; // flush keycode buffer
    }

    return true;
}

bool keypad_init (void)
{
    hal.delay_ms(510, NULL);

    if((connected = i2c_start().ok && i2c_probe(KEYPAD_I2CADDR) &&
        hal.irq_claim(IRQ_I2C_Strobe, 0, keypad_strobe_handler) &&
         (nvs_address = nvs_alloc(sizeof(jog_settings_t))))) {

        settings_register(&setting_details);

        if(keypad.on_jogmode_changed)
            keypad.on_jogmode_changed(jogdata.mode);
    }

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    return nvs_address != 0;
}

#else // KEYPAD_ENABLE == 2

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Keypad", KEYPAD_VERSION);
}

static ISR_CODE bool ISR_FUNC(keypad_enqueue_keycode)(char c)
{
    uint32_t bptr = (keybuf.head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

#if MPG_ENABLE && defined(MPG_STREAM) && MPG_STREAM != KEYPAD_STREAM
    if(c == CMD_MPG_MODE_TOGGLE)
        return true;
#endif

    if(c == CMD_JOG_CANCEL || (c == ASCII_CAN && !(state_get() & (STATE_ESTOP|STATE_ALARM)))) {
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
            task_add_immediate(keypad_process_keypress, NULL);
    }

    return true;
}

bool keypad_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(jog_settings_t)))) {

#if MPG_ENABLE && defined(MPG_STREAM) && MPG_STREAM == KEYPAD_STREAM
        if((hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(KEYPAD_STREAM, 115200, NULL, "MPG & Keypad"), false, keypad_enqueue_keycode))) {
#else
        if(stream_open_instance(KEYPAD_STREAM, 115200, keypad_enqueue_keycode, "Keypad")) {
#endif
            on_report_options = grbl.on_report_options;
            grbl.on_report_options = onReportOptions;

            settings_register(&setting_details);

            if(keypad.on_jogmode_changed)
                keypad.on_jogmode_changed(jogdata.mode);
        }
    }

    return nvs_address != 0;
}

#endif // KEYPAD_ENABLE == 2

#endif // KEYPAD_ENABLE
