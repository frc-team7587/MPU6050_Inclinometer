/*
 * VSPIPinAssignments.h
 *
 *  Created on: Jan 28, 2023
 *      Author: Metuchen Momentum, FIRST Robotics Team 7587
 *
 * VSPI, a.k.a. SPI3 native pin assignments.
 *
 * VSPI Pin Assignments
 * ---- --- -----------
 *
 * +-----+--------------------------------------------------------------------+
 * | Pin | Function                                                           |
 * +-----+--------------------------------------------------------------------+
 * |  23 | VSPI Master Out/Slave In (MOSI) (default) carries data from the    |
 * |     | the RoboRio (the master) to the ESP32 (the slave) .                |
 * +-----+--------------------------------------------------------------------+
 * |  19 | VSPI Master In/Slave Out (MISO) (default) carries data from the    |
 * |     | the ESP32 (the slave) to the RoboRio (the RoboRio).                |
 * +-----+--------------------------------------------------------------------+
 * |  18 | VSPI Clock (SCK) (default) carries the clock signal from the       |
 * |     | RoboRio to the ESP32. The computers exchange one bit per clock     |
 * |     | pulse.                                                             |
 * +-----+--------------------------------------------------------------------+
 * |   5 | VSPI Slave Select (SS) (default) how the RoboRio orders the ESP32  |
 * |     | to exchange data                                                   |
 * +-----+--------------------------------------------------------------------+
 */

#ifndef VSPIPINASSIGNMENTS_H_
#define VSPIPINASSIGNMENTS_H_

// Native pin assignments for VSPI, a.k.a. SPI3
#define GPIO_MOSI GPIO_NUM_23
#define GPIO_MISO GPIO_NUM_19
#define GPIO_SCLK GPIO_NUM_18
#define GPIO_SS GPIO_NUM_5

#endif /* VSPIPINASSIGNMENTS_H_ */
