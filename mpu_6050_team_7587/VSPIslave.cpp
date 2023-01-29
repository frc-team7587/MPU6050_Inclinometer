/*
 * VSPIslave.cpp
 *
 *  Created on: Jan 22, 2023
 *      Author: Metuchen Momentum, FIRST Robotics Team 7587
 */

#include "VSPIslave.h"

#include <stdlib.h>

static spi_bus_config_t spi_bus_configuration = {
  .mosi_io_num=MOSI,
  .miso_io_num=MISO,
  .sclk_io_num=SCK,
  .quadwp_io_num = -1,
  .quadhd_io_num = -1,};

/**
 * Ready to Transmit interrupt handler, which lowers SPI CLEAR TO SEND NOT and
 * raises SPI CLOSE SWITCH
 */
void IRAM_ATTR ISR_transaction_opened(spi_slave_transaction_t* transaction) {
  ISR_gpio::lower_gpio(((VSPI_slave::user_data *) transaction->user)
      ->select_acknowledge_pin);
  ISR_gpio::raise_gpio(((VSPI_slave::user_data *) transaction->user)
      ->close_switch_to_ack_pin);
}

/**
 * Transmission complete interrupt handler, which raises SPI CLEAR TO SEND NOT
 * and lowers SPI CLOSE SWITCH
 */
void IRAM_ATTR ISR_transaction_closed(spi_slave_transaction_t* transaction) {
  ISR_gpio::raise_gpio(((VSPI_slave::user_data *) transaction->user)
      ->select_acknowledge_pin);
  ISR_gpio::lower_gpio(((VSPI_slave::user_data *) transaction->user)
      ->close_switch_to_ack_pin);
}

VSPI_slave::VSPI_slave(
    gpio_num_t slave_select_pin,
    gpio_num_t select_acknowledge_pin,
    gpio_num_t close_switch_to_ack_pin,
    uint8_t mode) :
      slave_select_pin(slave_select_pin),
      select_acknowledge_pin(select_acknowledge_pin),
      close_switch_to_ack_pin(close_switch_to_ack_pin) {
  slave_configuration.spics_io_num = slave_select_pin;
  slave_configuration.flags = 0;
  slave_configuration.queue_size = 3;
  slave_configuration.mode = mode;
  slave_configuration.post_setup_cb = ISR_transaction_opened;
  slave_configuration.post_trans_cb = ISR_transaction_closed;
  memset(&slave_transaction, 0, sizeof(slave_transaction));
}

esp_err_t VSPI_slave::begin() {
  Serial.print("From VSPI_Slave::begin(), slave select: ");
  Serial.print(slave_configuration.spics_io_num);
  Serial.print(", mode: ");
  Serial.println(slave_configuration.mode);
  Serial.print("MOSI: ");
  Serial.print(spi_bus_configuration.mosi_io_num);
  Serial.print(", MISO: ");
  Serial.print(spi_bus_configuration.miso_io_num);
  Serial.print(", SCLK: ");
  Serial.println(spi_bus_configuration.sclk_io_num);

  return spi_slave_initialize(
      SPI3_HOST,
      &spi_bus_configuration,
      &slave_configuration,
      SPI_DMA_CH_AUTO);
}

esp_err_t VSPI_slave::send(
    const void * transmit_buffer,
    void * receive_buffer,
    const size_t buffer_length_in_bytes) {
  memset(&slave_transaction, 0, sizeof(slave_transaction));
  callback_data.select_acknowledge_pin = select_acknowledge_pin;
  callback_data.close_switch_to_ack_pin = close_switch_to_ack_pin;
  slave_transaction.length = 8 * buffer_length_in_bytes;
  slave_transaction.trans_len = 8 * buffer_length_in_bytes;
  slave_transaction.tx_buffer = transmit_buffer;
  slave_transaction.rx_buffer = receive_buffer;
  slave_transaction.user = &callback_data;

  esp_err_t status = spi_slave_transmit(SPI3_HOST, &slave_transaction, portMAX_DELAY);
  return status;
}
