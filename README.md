MegaAVR board definitions
=========================

Custom Arduino board definitions for ATMegaX09 boards.

This was created for a custom ATMega809 breakout board with access to more pins than
generally-available boards like the Arduino Nano Every.

Adding to Arduino IDE
---------------------

* Add the following URL to the `Additional Boards Manager URLs` in the IDE preferences
* Install `Gremblor MegaAVR Boards`

Programming
-----------

The MegaAVR 809 breakout board must be programmed over the UDPI port.

TODO: Describe UDPI programming process, using another Arduino as programmer.

Acknowledgements
----------------

This platform definition is based on the Arduino MegaAVR platform definition v1.8.7.

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
