## Keypad plugin

This plugin can be used for jogging, controlling feedhold, cycle start etc. via single character commands.
The command characters can be delivered either via an I2C or an UART port.

The plugin is enabled in _my_machine.h_ by removing the comment from the `#define KEYPAD_ENABLE` line and changing it if required:  
`#define KEYPAD_ENABLE 1` enables I2C mode, an additional strobe pin is required to signal keypresses.  
`#define KEYPAD_ENABLE 2` enables UART mode.

See the [core wiki](https://github.com/grblHAL/core/wiki/MPG-and-DRO-interfaces#keypad-plugin) for more details.

[Settings](https://github.com/terjeio/grblHAL/wiki/Additional-or-extended-settings#jogging) are provided for jog speed and distance for step, slow and fast jogging.

Character to action map:

|Character | Action                      |
|----------|-----------------------------|
| `M`      | Toggle Mist coolant output  |
| `C`      | Toggle Flood coolant output |
| `!`      | Enter feed hold             |
| `~`      | Cycle start                 |
| `0x8A`   | Toggle Fan 0 output<sup>1</sup> |
| `0x8B`   | Enable MPG full control<sup>2</sup> |
| `0x85`   | Cancel jog motion<sup>3</sup> |
| `0`      | Enable step mode jogging    |
| `1`      | Enable slow jogging mode    |
| `2`      | Enable fast jogging mode    |
| `h`      | Select next jog mode        |
| `H`      | Home machine                |
| `R`      | Continuous jog X+           |
| `L`      | Continuous jog X-           |
| `F`      | Continuous jog Y+           |
| `B`      | Continuous jog Y-           |
| `U`      | Continuous jog Z+           |
| `D`      | Continuous jog Z-           |
| `r`      | Continuous jog X+Y+         |
| `q`      | Continuous jog X+Y-         |
| `s`      | Continuous jog X-Y+         |
| `t`      | Continuous jog X-Y-         |
| `w`      | Continuous jog X+Z+         |
| `v`      | Continuous jog X+Z-         |
| `u`      | Continuous jog X-Z+         |
| `x`      | Continuous jog X-Z-         |

<sup>1</sup> The [fans plugin](https://github.com/grblHAL/Plugin_fans) is required.  
<sup>2</sup> Only available if MPG mode is enabled. Build 20220105 or later is required.  
<sup>3</sup> Only available in UART mode, it is recommended to send this on all key up events. In I2C mode the strobe line going high is used to signal jog cancel.

---

Dependencies:

An app providing input such as [this implementation](https://github.com/terjeio/I2C-interface-for-4x4-keyboard).

Driver (and app) must support I2C communication and a keypad strobe interrupt signal or have a free UART port depending on the mode selected.

---
2022-01-06
