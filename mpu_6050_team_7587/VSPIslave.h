/*
 * VSPIslave.h
 *
 *  Created on: Jan 22, 2023
 *      Author: Metuchen Momentum, FIRST Robotics Team 7587
 *
 * Performs an SPI slave transaction with handshake on VSPI (SPI3). The
 * handshake is
 *
 * 1. The master lowers Slave Select
 * 2. The slave senses that Slave Select has been lowered, stages the
 *    data to be sent, and opens a slave transaction.
 * 3. The slave transaction invokes an interrupt service routine which lowers
 *    an ACK pin. The slave also raises the "close switch on ready" pin to
 *    close a switch (actually an opto-isolator) on a RoboRio DIO pin
 * 4. The master performs the transfer, which exchanges data between the two
 *    machines.
 * 5. When the transfer completes, the slave invokes a callback that raises
 *    the ACK pin and lowers the "close switch on ready" pin to open a
 *    switch on a RoboRio DIO pin.
 *
 * Note that master and slave must use identical values for:
 *
 * 1. The SPI mode.
 * 2. The transfer length, which the ESP32 specifies in bits
 * 3. The transfer uses one MISO and one MOSI lines.
 *
 * The recommended transfer speed, which the master MUST NOT exceed, is
 * 1,000,000 Hz (1 MHz).
 *
 * Invoking applications must:
 *
 * 1. Configure the ACK pin as output
 * 2. Set the ACK pin HIGH
 * 3. Configure the "switch close on ready to send" pin as output.
 * 4. Set the "switch close on ready to send" pin LOW.
 */

#ifndef VSPISLAVE_H_
#define VSPISLAVE_H_

#include "Arduino.h"
#include "driver/spi_slave.h"

#include "ISRgpio.h"

class VSPI_slave {
public:
  struct user_data {
    gpio_num_t select_acknowledge_pin;
    gpio_num_t close_switch_to_ack_pin;
  };

private:
  const gpio_num_t slave_select_pin;
  const gpio_num_t select_acknowledge_pin;
  gpio_num_t close_switch_to_ack_pin;

  spi_slave_interface_config_t slave_configuration;
  spi_slave_transaction_t slave_transaction;

  user_data callback_data;

public:

  /**
   * Constructor
   *
   * Parameters:
   *
   * Name                    Contents
   * ----------------        ---------------------------------------------------
   * slave_seect_pin         A.K.A. SS, the receiver lowers this pin when it is
   *                         ready to receive data over SPI. Note that the data
   *                         must already be in the transmit buffer.
   * select_acknowlege pin   The sender lowers this pin to signal that it is
   *                         ready to send and that the receiver can start the
   *                         actual transfer.
   * close_switch_to_ack_pin Raised to close a switch when the slave is ready
   *                         to exchange data with the RoboRio. This pin must
   *                         be wired to an opto-isolator or similar device
   *                         that makes a connection when the pin goes high.
   * mode                    SPI Mode. Prefer mode 0.
   */
  VSPI_slave(
      gpio_num_t slave_select_pin,
      gpio_num_t select_acknowledge_pin,
      gpio_num_t close_switch_to_ack_pin,
      uint8_t mode);

  /**
   * Configures the VSPI bus. The application **must** invoke this before
   * performing any I/O. Note that the application **must** configure all
   * I/O pins before invoking this method or the SPI bus will not work, period,
   * the end. You can ask me how I know!
   *
   * Returns:
   *
   * ESP_ERR_INVALID_ARG if configuration is invalid
   * ESP_ERR_INVALID_STATE if host already is in use
   * ESP_ERR_NOT_FOUND if there is no available DMA channel
   * ESP_ERR_NO_MEM if out of memory
   * ESP_OK on success
   *
   * See: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_slave.html#_CPPv420spi_slave_initialize17spi_host_device_tPK16spi_bus_config_tPK28spi_slave_interface_config_t14spi_dma_chan_t
   */
  esp_err_t begin();

  /**
   * Sends information to the receiver, and, optionally, receive
   * information in return. Note that SPI communication is full duplex, with
   * the receiver optionally sending information to the sender.
   *
   * Parameters:
   *
   * Name                   Contents
   * ----------------       -----------------------------------------------------
   * transmit_buffer        Data to send to the receiver
   * receive_buffer         Data received from the receiver.
   * buffer_length_in_bytes The number of bytes available (i.e. length) of
   *                        BOTH buffers. See notes.
   * Notes:
   *
   * 1. Provide both buffers. Neither pointer can be NULL.
   * 2. The input buffer MUST be as long as the output buffer and vv.
   * 3. Buffers MUST contain 8 or more bytes and the number of bytes MUST
   *    be a multiple of 8.
   *
   * Returns:
   *
   * ESP_ERR_INVALID_ARG if parameter is invalid
   * ESP_OK on success
   *
   * see: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_slave.html#_CPPv421spi_slave_queue_trans17spi_host_device_tPK23spi_slave_transaction_t10TickType_t
   */
  esp_err_t send(
      const void * transmit_buffer,
      void * receive_buffer,
      const size_t buffer_length_in_bytes);

};

#endif /* VSPISLAVE_H_ */
