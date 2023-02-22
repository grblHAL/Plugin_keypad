/*
  display/i2c_leds.c - simple I2C status LED interface

  Part of grblHAL keypad plugins

  Copyright (c) 2023 Terje Io

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

#if I2C_ENABLE && DISPLAY_ENABLE == 2

#ifdef ARDUINO
#include "../../grbl/plugins.h"
#else
#include "grbl/plugins.h"
#endif

#ifndef DISPLAY2_PCA9654E
#define DISPLAY2_PCA9654E 1
#endif

#if DISPLAY2_PCA9654E
#define LEDS_I2CADDR (0x40 >> 1)
#define READ_INPUT    0
#define RW_OUTPUT     1
#define RW_INVERSION  2
#define RW_CONFIG     3
#endif

#ifndef LEDS_I2CADDR
#define LEDS_I2CADDR 0x49
#endif

typedef union {
    uint8_t value;
    struct {
        uint8_t led0: 1,
                led1: 1,
                led2: 1,
                led3: 1,
                led4: 1,
                led5: 1,
                led6: 1,
                led7: 1;
    };
    struct {
        uint8_t run:     1,
                hold:    1,
                spindle: 1,
                flood:   1,
                mist:    1,
                red:     1,
                green:   1,
                blue:    1;
    };
} leds_t;

static leds_t leds = {0};
static spindle_set_state_ptr spindle_set_state_;
static coolant_set_state_ptr coolant_set_state_;
static on_state_change_ptr on_state_change;
static on_report_options_ptr on_report_options;
static on_spindle_select_ptr on_spindle_select;

static void leds_write (leds_t leds)
{
#if DISPLAY2_PCA9654E
    uint8_t cmd[2];

    cmd[0] = RW_OUTPUT;
    cmd[1] = leds.value;

    i2c_send(LEDS_I2CADDR, cmd, 2, false);
#else
    i2c_send(LEDS_I2CADDR, leds.value, 1, false);
#endif
}

static void onStateChanged (sys_state_t state)
{
    static sys_state_t last_state = STATE_IDLE;

    if(state != last_state) {
        last_state = state;
        leds.run = state == STATE_CYCLE;
        leds.hold = state == STATE_HOLD;
        leds_write(leds);
    }

    if(on_state_change)
        on_state_change(state);
}

static void onSpindleSetState (spindle_state_t state, float rpm)
{
    spindle_set_state_(state, rpm);

    leds.spindle = state.on;
    leds_write(leds);
}

static void onCoolantSetState (coolant_state_t state)
{
    coolant_set_state_(state);

    leds.flood = state.flood;
    leds.mist = state.mist;
    leds_write(leds);
}

static bool onSpindleSelect (spindle_ptrs_t *spindle)
{
    spindle_set_state_ = spindle->set_state;
    spindle->set_state = onSpindleSetState;

    return on_spindle_select == NULL || on_spindle_select(spindle);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:I2C LEDS v0.02]" ASCII_EOL);
}

static void warn_unavailable (sys_state_t state)
{
    report_message("I2C LEDs not connected!", Message_Warning);
}

void display_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    if(i2c_probe(LEDS_I2CADDR)) {

        on_state_change = grbl.on_state_change;
        grbl.on_state_change = onStateChanged;

        on_spindle_select = grbl.on_spindle_select;
        grbl.on_spindle_select = onSpindleSelect;

        coolant_set_state_ = hal.coolant.set_state;
        hal.coolant.set_state = onCoolantSetState;

#if DISPLAY2_PCA9654E
        uint8_t cmd[2];

        cmd[0] = RW_CONFIG;
        cmd[1] = 0;
        i2c_send(LEDS_I2CADDR, cmd, 2, true);

        cmd[0] = RW_INVERSION;
        cmd[1] = 0;
        i2c_send(LEDS_I2CADDR, cmd, 2, true);
#endif

    } else
        protocol_enqueue_rt_command(warn_unavailable);
}

#endif
