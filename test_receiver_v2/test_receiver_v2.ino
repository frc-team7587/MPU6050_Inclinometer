/**
 * A stand-in for the RoboRio's SPI interface.
 */
#include "Arduino.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

// Pin definitions
#define LED_PIN 2  // On-board LED
#define SLAVE_READY_NOT 4  // LOW <--> slave ready.
#define SLAVE_SELECT 5  // LOW <--> select slave.
#define SPI_CLEAR_TO_SEND_NOT 16  // LOW --> master can send.
#define ROBO_RIO_WANTS_DATA_NOT 17  // RoboRio pulls this LOW to get data.
#define ROBO_RIO_WANTS_DATA_FWD_RELAY GPIO_NUM_26  // Data request relay closure
#define GREEN_LED 12
#define YELLOW_LED 14
#define RED_LED 27

#define GPIO_MISO GPIO_NUM_19
#define GPIO_MOSI GPIO_NUM_23
#define GPIO_SCLK GPIO_NUM_18
#define GPIO_SS GPIO_NUM_5

#define BUFFER_LENGTH 16
#define SPI_BITS_TO_SEND 8 * BUFFER_LENGTH

#define TEST_CHARACTER 0b01010101

DMA_ATTR WORD_ALIGNED_ATTR uint8_t receive_buffer[BUFFER_LENGTH];
DMA_ATTR WORD_ALIGNED_ATTR uint8_t transmit_buffer[BUFFER_LENGTH];

spi_bus_config_t spi_bus_configuration = {
    .mosi_io_num=MOSI,
    .miso_io_num=MISO,
    .sclk_io_num=SCK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 32,
};

spi_device_interface_config_t devcfg = {
     .command_bits=0,
     .address_bits=0,
     .dummy_bits=0,
     .mode=SPI_MODE0,
     .duty_cycle_pos=128,        //50% duty cycle
     .cs_ena_pretrans = 0,
     .cs_ena_posttrans=20,        //Keep the CS low 20 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
     .clock_speed_hz=SPI_MASTER_FREQ_8M / 80,
     .input_delay_ns = 0,
     .spics_io_num=GPIO_SS,
     .queue_size=3
 };

spi_transaction_t spi_transaction;

spi_device_handle_t h_gyroscope_device;

int dummy_count = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("MCU6050-based pitch angle RoboRio emulator v2.0 Beta compiled on ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);

  pinMode(LED_PIN, OUTPUT);
  pinMode(SLAVE_READY_NOT, INPUT_PULLUP);
  pinMode(SPI_CLEAR_TO_SEND_NOT, INPUT_PULLUP);
  pinMode(ROBO_RIO_WANTS_DATA_NOT, OUTPUT);
  pinMode(ROBO_RIO_WANTS_DATA_FWD_RELAY, OUTPUT);

  pinMode(GPIO_MOSI, OUTPUT);
  pinMode(GPIO_MISO, INPUT_PULLUP);
  pinMode(GPIO_SCLK, OUTPUT);
  pinMode(GPIO_SS, OUTPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(GPIO_SS, HIGH);
  digitalWrite(ROBO_RIO_WANTS_DATA_NOT, HIGH);
  digitalWrite(ROBO_RIO_WANTS_DATA_FWD_RELAY, LOW);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(RED_LED, HIGH);
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(YELLOW_LED, HIGH);
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(GREEN_LED, HIGH);
  vTaskDelay(pdMS_TO_TICKS(1000));

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);

  if (spi_bus_initialize(
      SPI3_HOST, &spi_bus_configuration, SPI_DMA_CH_AUTO) != ESP_OK) {
    error_loop(1);
  }
  Serial.println("SPI bus initialized.");

  if (spi_bus_add_device(SPI3_HOST, &devcfg, &h_gyroscope_device) != ESP_OK) {
    error_loop(2);
  }

  digitalWrite(RED_LED, HIGH);
  vTaskDelay(pdMS_TO_TICKS(5000));  // Give slave time to initialize
  digitalWrite(RED_LED, LOW);
}

void error_loop(int code) {
  for (;;) {
    for (int i = 0; i < code; ++i) {
      digitalWrite(BUILTIN_LED, HIGH);
      vTaskDelay(pdMS_TO_TICKS(150));
      digitalWrite(BUILTIN_LED, LOW);
      vTaskDelay(pdMS_TO_TICKS(150));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void send_spi() {
  memset(receive_buffer, 0, sizeof(receive_buffer));
  memset(transmit_buffer, 0, sizeof(transmit_buffer));
  memset(&spi_transaction, 0, sizeof(spi_transaction));

  for (int i = 0; i < BUFFER_LENGTH - 4; ++i) {
    transmit_buffer[i] = TEST_CHARACTER;
  }

  spi_transaction.rx_buffer = receive_buffer;
  spi_transaction.tx_buffer = transmit_buffer;
  spi_transaction.length = SPI_BITS_TO_SEND;

  digitalWrite(ROBO_RIO_WANTS_DATA_NOT, LOW);
  digitalWrite(ROBO_RIO_WANTS_DATA_FWD_RELAY, HIGH);

  if (spi_device_acquire_bus(h_gyroscope_device, portMAX_DELAY) != ESP_OK) {
    error_loop(3);
  }
  digitalWrite(YELLOW_LED, HIGH);

  // Note: spin loops are normally discouraged because long delays can
  // trigger FreeRTOS's watchdog timer. However, the delay is expected to
  // be short, no more than a few machine cycles, so it should be OK.
  while (digitalRead(SPI_CLEAR_TO_SEND_NOT) == HIGH) {
  }
  digitalWrite(GREEN_LED, HIGH);

  while(digitalRead(SPI_CLEAR_TO_SEND_NOT) == HIGH) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (spi_device_transmit(h_gyroscope_device, &spi_transaction) != ESP_OK) {
    error_loop(4);
  }

  digitalWrite(GREEN_LED, LOW);
  while(digitalRead(SPI_CLEAR_TO_SEND_NOT) == LOW) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  spi_device_release_bus(h_gyroscope_device);
  digitalWrite(YELLOW_LED, LOW);

  digitalWrite(ROBO_RIO_WANTS_DATA_NOT, HIGH);
  digitalWrite(ROBO_RIO_WANTS_DATA_FWD_RELAY, LOW);

  Serial.print("Received: ");
  Serial.print((const char *) receive_buffer);
  long angle = strtol((const char *) receive_buffer, NULL, 16);
  Serial.print(", in tenths of a degree: ");
  Serial.println(angle);
}


// The loop function is called in an endless loop
void loop() {
  vTaskDelay(pdMS_TO_TICKS(5000));  // 5000 for humans, 10 for 'scope
  send_spi();
}
