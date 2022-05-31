/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>
#include "timers.h"

#define NUM_DIGITAL_PINS            21 // D0..D23 minus MOSI/MISO/SCK
#define NUM_ANALOG_INPUTS           10
#define NUM_RESERVED_PINS           1  // (RESET_L)
#define NUM_INTERNALLY_USED_PINS    0  //
#define NUM_I2C_PINS                2  // (SDA / SCL)
#define NUM_SPI_PINS                3  // (MISO / MOSI / SCK)
#define NUM_TOTAL_FREE_PINS         (NUM_DIGITAL_PINS)
#define NUM_TOTAL_PINS              (NUM_DIGITAL_PINS + NUM_RESERVED_PINS + NUM_INTERNALLY_USED_PINS + NUM_I2C_PINS + NUM_SPI_PINS)
#define ANALOG_INPUT_OFFSET         10

#define EXTERNAL_NUM_INTERRUPTS     48

#define digitalPinHasPWM(p)         ((p) == 2 || (p) == 3 || ((p) >= 8 && (p) <= 15))

#define SPI_MUX       (PORTMUX_SPI0_ALT2_gc)
#define PIN_SPI_MISO  (17)
#define PIN_SPI_SCK   (18)
#define PIN_SPI_MOSI  (16)
#define PIN_SPI_SS    (19)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA  (25)
#define PIN_WIRE_SCL  (26)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// Main USART available on Arduino header pins
// USART0 on mega4809 (default pins)
// Mapped to HWSERIAL0 in Serial library
#define HWSERIAL1 (&USART0)
#define HWSERIAL1_DRE_VECTOR (USART0_DRE_vect)
#define HWSERIAL1_DRE_VECTOR_NUM (USART0_DRE_vect_num)
#define HWSERIAL1_RXC_VECTOR (USART0_RXC_vect)
#define HWSERIAL1_MUX (PORTMUX_USART0_DEFAULT_gc)
#define PIN_WIRE_HWSERIAL1_RX (1)
#define PIN_WIRE_HWSERIAL1_TX (0)

// Uno2 Debug USART
// USART3 on mega4809 (default pins)
// Mapped to HWSERIAL1 in Serial library
#define HWSERIAL0 (&USART3)
#define HWSERIAL0_DRE_VECTOR (USART3_DRE_vect)
#define HWSERIAL0_DRE_VECTOR_NUM (USART3_DRE_vect_num)
#define HWSERIAL0_RXC_VECTOR (USART3_RXC_vect)
#define HWSERIAL0_MUX (PORTMUX_USART3_DEFAULT_gc)
#define PIN_WIRE_HWSERIAL0_RX (7)
#define PIN_WIRE_HWSERIAL0_TX (6)

// HWSERIAL2 mapped to USART1
#define HWSERIAL2 (&USART1)
#define HWSERIAL2_DRE_VECTOR (USART1_DRE_vect)
#define HWSERIAL2_DRE_VECTOR_NUM (USART1_DRE_vect_num)
#define HWSERIAL2_RXC_VECTOR (USART1_RXC_vect)
#define HWSERIAL2_MUX (PORTMUX_USART1_DEFAULT_gc)
#define PIN_WIRE_HWSERIAL2_RX (9)
#define PIN_WIRE_HWSERIAL2_TX (8)

#define HWSERIAL3_MUX (PORTMUX_USART2_NONE_gc)

#define TWI_MUX (PORTMUX_TWI0_ALT2_gc) // (PORTMUX_TWI0_DEFAULT_gc)

#define MUX_SPI (SPI_MUX)
#define SPI_INTERFACES_COUNT 1

#define LED_BUILTIN   (23)

#define PIN_A0   (10)
#define PIN_A1   (11)
#define PIN_A2   (12)
#define PIN_A3   (13)
#define PIN_A4   (14)
#define PIN_A5   (15)
#define PIN_A6   (16)
#define PIN_A7   (17)
#define PIN_A8   (18)
#define PIN_A9   (19)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;
static const uint8_t A8 = PIN_A8;
static const uint8_t A9 = PIN_A9;

#define PINS_COUNT    (40u)

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// TODO(aaron): This graphic does NOT reflect the Gremega809 breakout digital or analog pin
// assignments. Refer to PDF for proper pin mapping.
//
// ATMEGA4809 / ARDUINO
//
//                     (SCL)(SDA) (7)  (2)                 (R)  (3~) (6~)
//                 PA4  PA3  PA2  PA1  PA0  GND  VDD  UPDI PF6  PF5  PF4  PF3
//
//                  48   47   46   45   44   43   42   41   40   39   38   37
//              + ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ +
//        PA5   1|                                                             |36  PF2
//        PA6   2|                                                             |35  PF1 (TOSC2)
//        PA7   3|                                                             |34  PF0 (TOSC1)
//   (9~) PB0   4|                                                             |33  PE3 (8)
//  (10~) PB1   5|                                                             |32  PE2 (13)
//   (5~) PB2   6|                                                             |31  PE1 (12)
//        PB3   7|                          48pin QFN                          |30  PE0 (11~)
//   (Tx) PB4   8|                                                             |29  GND
//   (Rx) PB5   9|                                                             |28  AVDD
//        PC0  10|                                                             |27  PD7 (VREF)
//        PC1  11|                                                             |26  PD6
//        PC2  12|                                                             |25  PD5 (A5)
//               + ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ ____ +
//                  13   14   15   16   17   18   19   20   21   22   23   24
//
//                  PC3  VDD  GND  PC4  PC5  PC6  PC7  PD0  PD1  PD2  PD3  PD4
//                                 (1)  (0)  (4)       (A0) (A1) (A2) (A3) (A4)

//

const uint8_t digital_pin_to_port[] = {
  PA, // 0 PA0/USART0_Tx
  PA, // 1 PA1/USART0_Rx
  PA, // 2 PA2
  PA, // 3 PA3
  PA, // 4 PA6
  PA, // 5 PA7
  PB, // 6 PB0/USART3_Tx
  PB, // 7 PB1/USART3_Rx
  PC, // 8 PC0/USART1_Tx
  PC, // 9 PC1/USART1_Rx
  PD, // 10 PD0/AI0
  PD, // 11 PD1/AI1
  PD, // 12 PD2/AI2
  PD, // 13 PD3/AI3
  PD, // 14 PD4/AI4
  PD, // 15 PD5/AI5
  PE, // 16 PE0/AI8/MOSI
  PE, // 17 PE1/AI9/MISO
  PE, // 18 PE2/AI10/SCK
  PE, // 19 PE3/AI11/SS
  PF, // 20 PF0
  PF, // 21 PF1
  PF, // 22 PF2
  PF, // 23 PF3/LED
  PF, // 24 PF6/RESET_L
  PC, // 25 PC2/TWI_SDA
  PC, // 26 PC3/TWI_SCL
};

/* Use this for accessing PINnCTRL register */
const uint8_t digital_pin_to_bit_position[] = {
  PIN0_bp,  // 0 PA0/USART0_Tx
  PIN1_bp,  // 1 PA1/USART0_Rx
  PIN2_bp,  // 2 PA2
  PIN3_bp,  // 3 PA3
  PIN6_bp,  // 4 PA6
  PIN7_bp,  // 5 PA7
  PIN0_bp,  // 6 PB0/USART3_Tx
  PIN1_bp,  // 7 PB1/USART3_Rx
  PIN0_bp,  // 8 PC0/USART1_Tx
  PIN1_bp,  // 9 PC1/USART1_Rx
  PIN0_bp,  // 10 PD0/AI0
  PIN1_bp,  // 11 PD1/AI1
  PIN2_bp,  // 12 PD2/AI2
  PIN3_bp,  // 13 PD3/AI3
  PIN4_bp,  // 14 PD4/AI4
  PIN5_bp,  // 15 PD5/AI5
  PIN0_bp,  // 16 PE0/AI8/MOSI
  PIN1_bp,  // 17 PE1/AI9/MISO
  PIN2_bp,  // 18 PE2/AI10/SCK
  PIN3_bp,  // 19 PE3/AI11/SS
  PIN0_bp,  // 20 PF0
  PIN1_bp,  // 21 PF1
  PIN2_bp,  // 22 PF2
  PIN3_bp,  // 23 PF3/LED
  PIN6_bp,  // 24 PF6/RESET_L
  PIN2_bp,  // 25 PC2/TWI_SDA
  PIN3_bp,  // 26 PC3/TWI_SCL
};

/* Use this for accessing PINnCTRL register */
const uint8_t digital_pin_to_bit_mask[] = {
  PIN0_bm,  // 0 PA0/USART0_Tx
  PIN1_bm,  // 1 PA1/USART0_Rx
  PIN2_bm,  // 2 PA2
  PIN3_bm,  // 3 PA3
  PIN6_bm,  // 4 PA6
  PIN7_bm,  // 5 PA7
  PIN0_bm,  // 6 PB0/USART3_Tx
  PIN1_bm,  // 7 PB1/USART3_Rx
  PIN0_bm,  // 8 PC0/USART1_Tx
  PIN1_bm,  // 9 PC1/USART1_Rx
  PIN0_bm,  // 10 PD0/AI0
  PIN1_bm,  // 11 PD1/AI1
  PIN2_bm,  // 12 PD2/AI2
  PIN3_bm,  // 13 PD3/AI3
  PIN4_bm,  // 14 PD4/AI4
  PIN5_bm,  // 15 PD5/AI5
  PIN0_bm,  // 16 PE0/AI8/MOSI
  PIN1_bm,  // 17 PE1/AI9/MISO
  PIN2_bm,  // 18 PE2/AI10/SCK
  PIN3_bm,  // 19 PE3/AI11/SS
  PIN0_bm,  // 20 PF0
  PIN1_bm,  // 21 PF1
  PIN2_bm,  // 22 PF2
  PIN3_bm,  // 23 PF3/LED
  PIN6_bm,  // 24 PF6/RESET_L
  PIN2_bm,  // 25 PC2/TWI_SDA
  PIN3_bm,  // 26 PC3/TWI_SCL
};

const uint8_t digital_pin_to_timer[] = {
  NOT_ON_TIMER,  // 0 PA0/USART0_Tx
  NOT_ON_TIMER,  // 1 PA1/USART0_Rx
  TIMERB0,       // 2 PA2
  TIMERB1,       // 3 PA3
  NOT_ON_TIMER,  // 4 PA6
  NOT_ON_TIMER,  // 5 PA7
  NOT_ON_TIMER,  // 6 PB0/USART3_Tx
  NOT_ON_TIMER,  // 7 PB1/USART3_Rx
  TIMERB2,       // 8 PC0/USART1_Tx
  TIMERB3,       // 9 PC1/USART1_Rx
  TIMERA0,       // 10 PD0/AI0
  TIMERA0,       // 11 PD1/AI1
  TIMERA0,       // 12 PD2/AI2
  TIMERA0,       // 13 PD3/AI3
  TIMERA0,       // 14 PD4/AI4
  TIMERA0,       // 15 PD5/AI5
  NOT_ON_TIMER,  // 16 PE0/AI8/MOSI
  NOT_ON_TIMER,  // 17 PE1/AI9/MISO
  NOT_ON_TIMER,  // 18 PE2/AI10/SCK
  NOT_ON_TIMER,  // 19 PE3/AI11/SS
  NOT_ON_TIMER,  // 20 PF0
  NOT_ON_TIMER,  // 21 PF1
  NOT_ON_TIMER,  // 22 PF2
  NOT_ON_TIMER,  // 23 PF3/LED
  NOT_ON_TIMER,  // 24 PF6/RESET_L
  NOT_ON_TIMER,  // 25 PC2/TWI_SDA
  NOT_ON_TIMER,  // 26 PC3/TWI_SCL
};

const uint8_t analog_pin_to_channel[] = {
  0,
  1,
  2,
  3,
  4,
  5,
  8,
  9,
  10,
  11,
};

#endif

#define digitalPinToAnalogInput(p)  ((p < ANALOG_INPUT_OFFSET) ? analog_pin_to_channel[p] : analog_pin_to_channel[p - ANALOG_INPUT_OFFSET] )

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR       Serial
#define SERIAL_PORT_HARDWARE      Serial1
#define SERIAL_PORT_USBVIRTUAL    Serial

#endif
