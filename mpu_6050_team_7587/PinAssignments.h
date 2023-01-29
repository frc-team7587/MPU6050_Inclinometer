/*
 * PinAssignments.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Metuchen Momentum, FIRST Robotics Team 7587
 *
 * Pins used by Team 7587's MCU-6050 inclinometer. Note that pins are
 * defined using the native ESP32 GPIO enumeration values.
 *
 * Pins used to drive the the test receiver
 *
 * GPIO Pin Assignments
 * ---- --- -----------
 *
 * +-----+--------------------------------------------------------------------+
 * | Pin | Function                                                           |
 * +-----+--------------------------------------------------------------------+
 * |  33 | Back yellow LED, which illuminates when the front is pitched down  |
 * |     | more than 6 and less than or equal to 10 degrees.                  |
 * +-----+--------------------------------------------------------------------+
 * |  32 | Back  red LED, which illuminates when the front is pitched down    |
 * |     | more than 10 degrees.                                              |
 * +-----+--------------------------------------------------------------------+
 * |  27 | Front green led, which illuminates when the front is pitched up    |
 * |     | more than 2 and less than or equal to 6 degrees.                   |
 * +-----+--------------------------------------------------------------------+
 * |  26 | Blue LED, which illuminates when the pitch is between -2 and 2     |
 * |     | degrees.                                                           |
 * +-----+--------------------------------------------------------------------+
 * |  25 | Back green LED, which illuminates when the front is pitched down   |
 * |     | more than 2 and less than or equal to 6 degrees.                   |
 * +-----+--------------------------------------------------------------------+
 * |  23 | VSPI Master Out/Slave In (MOSI) (default) carries data from the    |
 * |     | the RoboRio (the master) to the ESP32 (the slave) .                |
 * +-----+--------------------------------------------------------------------+
 * |  22 | I2C clock signal (SCL), used to communicate with the LCD display   |
 * |     | Since this is the default, it doesn't need to be specified.        |
 * +-----+--------------------------------------------------------------------+
 * |  21 | I2C data signal (SDA), used to communicate with the LCD display    |
 * |     | Since this is the default, it doesn't need to be specified.        |
 * +-----+--------------------------------------------------------------------+
 * |  19 | VSPI Master In/Slave Out (MISO) (default) carries data from the    |
 * |     | the ESP32 (the slave) to the RoboRio (the RoboRio).                |
 * +-----+--------------------------------------------------------------------+
 * |  18 | VSPI Clock (SCK) (default) carries the clock signal from the       |
 * |     | RoboRio to the ESP32. The computers exchange one bit per clock     |
 * |     | pulse.                                                             |
 * +-----+--------------------------------------------------------------------+
 * |  16 | SPI Master Clear To Send Not. The slave lowers this when the       |
 * |     | slave becomes ready to participate in a data exchange after the    |
 * |     | master lowers Slave Select.                                        |
 * +-----+--------------------------------------------------------------------+
 * |  14 | Drives the front yellow LED, which illuminates when the front is   |
 * |     | pitched up more than 6 and less than 10 degrees.                   |
 * +-----+--------------------------------------------------------------------+
 * |  13 | Drives the front red LED, which illuminates when the front has     |
 * |     | pitched up 10 or more degrees                                      |
 * +-----+--------------------------------------------------------------------+
 * |  12 | Close Switch When SPI Clear To Send, the compliment of SPI Master  |
 * |     | Clear To Send, which drives an opto-isolator that is connected to  |
 * |     | a RoboRio DIO pin. The RoboRio will sense a switch closure when    |
 * |     | the gyro is ready to perform an SPI data exchange.                 |
 * +-----+--------------------------------------------------------------------+
 * |   5 | VSPI Slave Select (SS) (default) how the RoboRio orders the ESP32  |
 * |     | to exchange data                                                   |
 * +-----+--------------------------------------------------------------------+
 * |   4 | /Slave Ready the ESP32 lowers this pin to inform the RoboRio that  |
 * |     | it has started successfully and can send the pitch when asked.     |
 * +-----+--------------------------------------------------------------------+
 */

#ifndef PINASSIGNMENTS_H_
#define PINASSIGNMENTS_H_

#include "driver/gpio.h"

// Angle indication LEDs
#define FRONT_RED_LED GPIO_NUM_13
#define FRONT_YELLOW_LED GPIO_NUM_14
#define FRONT_GREEN_LED GPIO_NUM_27
#define BLUE_LED GPIO_NUM_26
#define BACK_GREEN_LED GPIO_NUM_25
#define BACK_YELLOW_LED GPIO_NUM_33
#define BACK_RED_LED GPIO_NUM_32

// Handshake lines
#define SLAVE_READY_NOT GPIO_NUM_4  // LOW <--> slave ready.
#define SPI_CLEAR_TO_SEND_NOT GPIO_NUM_16  // LOW --> master can send.
#define ROBO_RIO_WANTS_DATA_NOT GPIO_NUM_39  // RoboRio pulls this LOW to get data.
#define SPI_CLEAR_TO_SEND_SWITCH GPIO_NUM_12

// The system will stop when this pin goes high. Wire a button to ground
// on one side and to a 10K pullup resistor on the pin side.
#define EMERGENCY_STOP_PIN GPIO_NUM_36

// Used for debugging.
#define INDICATOR_LED GPIO_NUM_17

// The on-board LED.
#define BUILTIN_LED GPIO_NUM_2

#endif /* PINASSIGNMENTS_H_ */
