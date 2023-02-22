/*
  keypad.h - I2C keypad plugin

  Part of grblHAL keypad plugins

  Copyright (c) 2017-2023 Terje Io

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

#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#ifdef ARDUINO
#include "../grbl/gcode.h"
#include "../grbl/settings.h"
#else
#include "grbl/gcode.h"
#include "grbl/settings.h"
#endif

#define KEYBUF_SIZE 8 // must be a power of 2
#ifndef KEYPAD_I2CADDR
#define KEYPAD_I2CADDR 0x49
#endif

#define JOG_XR   'R'
#define JOG_XL   'L'
#define JOG_YF   'F'
#define JOG_YB   'B'
#define JOG_ZU   'U'
#define JOG_ZD   'D'
#define JOG_XRYF 'r'
#define JOG_XRYB 'q'
#define JOG_XLYF 's'
#define JOG_XLYB 't'
#define JOG_XRZU 'w'
#define JOG_XRZD 'v'
#define JOG_XLZU 'u'
#define JOG_XLZD 'x'
#if N_AXIS > 3
#define JOG_AR   'A'
#define JOG_AL   'a'
#endif

typedef enum {
    JogMode_Fast = 0,
    JogMode_Slow,
    JogMode_Step
} jogmode_t;

typedef struct {
    jog_settings_t settings;
    float modifier[3];
    uint_fast8_t modifier_index;
    jogmode_t mode;
} jogdata_t;

typedef bool (*on_keypress_preview_ptr)(const char c, uint_fast16_t state);
typedef void (*on_jogmode_changed_ptr)(jogmode_t jogmode);
typedef void (*on_jogdata_changed_ptr)(jogdata_t *jogdata);

typedef struct {
    on_keypress_preview_ptr on_keypress_preview;
    on_jogmode_changed_ptr on_jogmode_changed;
    on_jogdata_changed_ptr on_jogdata_changed;
} keypad_t;

extern keypad_t keypad;

bool keypad_init (void);
bool keypad_enqueue_keycode (char c);

#endif
