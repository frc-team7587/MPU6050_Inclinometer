/*
 * ISRgpio.cpp
 *
 *  Created on: Jan 22, 2023
 *      Author: Metuchen Momentum, FIRST Robotics Team 7587
 */

#include "ISRgpio.h"


void IRAM_ATTR ISR_gpio::lower_gpio(gpio_num_t pin) {
  if ( pin < 32 )
      GPIO.out_w1tc = ((uint32_t)1 << pin);
  else if ( pin < 34 )
      GPIO.out1_w1tc.val = ((uint32_t)1 << (pin - 32));
}

void IRAM_ATTR ISR_gpio::raise_gpio(gpio_num_t pin) {
  if ( pin < 32 )
      GPIO.out_w1ts = ((uint32_t)1 << pin);
  else if ( pin < 34 )
      GPIO.out1_w1ts.val = ((uint32_t)1 << (pin - 32));
}
