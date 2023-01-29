/*
 * ISRgpio.h
 *
 *  Created on: Jan 22, 2023
 *      Author: Metuchen Momentum, FIRST Robotics Team 7587
 *
 * Low-level GPIO pin API for use by interrupt service routines. Neither the
 * Arduino nor ESP32 HAL libraries work when invoked by an ISR
 */

#ifndef ISRGPIO_H_
#define ISRGPIO_H_

#include "driver/gpio.h"

class ISR_gpio {
public:
  /**
   * Sets the specified GPIO pin to a nominal 3.3V. This is equivalent to
   * invoking the Arduino library method
   *
   *   digitalWrite(pin, HIGH);
   */
  static void IRAM_ATTR lower_gpio(gpio_num_t pin);

  /**
   * Sets the specified GPIO pin to a nominal 0V. This is equivalent to
   * invoking the Arduino library method
   *
   *   digitalWrite(pin, LOW);
   */
  static void IRAM_ATTR raise_gpio(gpio_num_t pin);
};

#endif /* ISRGPIO_H_ */
