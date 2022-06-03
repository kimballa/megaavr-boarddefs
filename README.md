MegaAVR board definitions
=========================

Custom Arduino board definitions for ATMegaX09 boards.

This was created for a [custom ATMega809 breakout
board](https://github.com/kimballa/ATMega809-breakout) with access to more pins than
generally-available boards like the Arduino Nano Every.

Adding to Arduino IDE
---------------------

* Add the following URL to the `Additional Boards Manager URLs` in the IDE preferences:
  `https://raw.githubusercontent.com/kimballa/arduino-pkg-index/main/package_gremblor_index.json`
* Install `Gremblor MegaAVR Boards`
* Select `Gremblor megaAVR 809 Breakout Board` in the `Tools > Board` menu.

Programming
-----------

The MegaAVR 809 breakout board must be programmed over the UDPI port.

There are two basic ways to access this:
* You can use a USB-to-serial adapter to create a serial connection directly from the
  programming computer to the MCU. Tie the RX and TX lines together with a shottky diode
  from TX to RX or a 470R resistor. (See [Spence's
  guidance](https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md)
  for more instructions.)
* Use an external programmer device. You can make one from another Arduino.
  * [jtag2updi](https://github.com/ElTangas/jtag2updi) is one of the most heavily-referenced. This
    claims to work on a broad variety of Arduinos, but the ATMega328 (Arduino Uno) seems to be
    the preferred system. I tried to get this to work on an Arduino Nano Every (ATMega4809) but
    couldn't get successful programming.
  * [microUPDI](https://github.com/MCUdude/microUPDI) is designed for a UDPI programmer built on
    the SparkFun Arduino Pro Micro, an ATMega32u4-based device. This has the same MCU and pin-out
    as an Arduino Leonardo. Follow the instructions to flash the Leonardo board with the firmware.
    Then connect the ATMega809 breakout's UDPI pin to pin D7 on the Leonardo (and connect GND and
    +5V).

If using a programmer,  then select "Upload using programmer" instead of "Upload" in the Sketch
menu. You can select the appropriate programmer in `Tools > Programmer`; definitions are provided for
each of the two programmer firmwares above.

Known issues
------------

The internal timer does not seem to work. This affects the `millis()` function as well as `delay()`.
The latter will freeze indefinitely.

There is likely some issue in the
[variant.c](https://github.com/kimballa/megaavr-boarddefs/blob/main/variants/gremega809/variant.c)
or
[pins\_arduino.h](https://github.com/kimballa/megaavr-boarddefs/blob/main/variants/gremega809/pins_arduino.h)
file.

Acknowledgements
----------------

This platform definition is based on the [Arduino MegaAVR platform
definition](https://github.com/arduino/ArduinoCore-megaavr) v1.8.7.

Such work is redistributed under the terms of the GNU Lesser General Public License,
with the following copyright acknowledgement:

```
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
```
