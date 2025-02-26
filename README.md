# Watchdog for TK102B tracker

The idea was taken from https://www.youtube.com/watch?v=dvGj_IYlURU

The device does not power on if power is restored.
If no ping or events on the input pin does a reset on the output pin.

# Pinout

MCU is STM8S001J3


| Pin | STM8S001J3 | STM8L001J3M3 |
|-----|------------|--------------|
| GND |     2      |     3        |
| VDD |     4      |     4        |
| IN  |     1      |     1        |
| OUT |     8      |     8        |

STM8S device require an additional external capacitor for a voltage regulator connected through the VCAP pin (for example, 1uF or 2.2uF external cap).
Connect between 2 and 3 pin.

## Useful links
- [TK102-v02 Mini GPS Auto Power Off bug Fix / Hack. tk102b](https://www.youtube.com/watch?v=dvGj_IYlURU)
- [Watchdog per GPS locator TK102 v3](https://forum.arduino.cc/t/watchdog-per-gps-locator-tk102-v3/567379/23)
