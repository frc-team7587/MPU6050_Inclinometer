# Test Driver/RoboRio Emulator for the 2023 Inclinometer

An ESP32s sketch that interacts with Metuchen Momentum's
inclinometer that supports the FIRST 2023 balance challenge.

## Background

Metuchen Momentum, FIRST Team 7587, has built an inclinometer so the
robot can balance automatically as it meets  the 2023 season's 
balance challenge. The resulting hardware/software package needs
a test harness to verify it. The sofware in the containing
project implements the required functionality.

The target system is an ESP32s2 microntroller.

## Hardware

The supporting hardware consists of

1. An ESP32S2 development board.
2. Three LEDs, red, yellow, and green, each of which requires
   a 512 Ohm current limiting resistor.
   
The test harness communicates with the inclinometer via
VSPI, a.k.a. SPI 3. Please consult the
[SPI Pinout](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
for details. VSPI is configured as follows:

| Setting | Value |
| ------- | ----- |
| SPI Mode | 0 (Zero) |
| Buffer Length | 16 bytes (see note) |
| Speed | 1 MHz |


Note: the ESP32 SPI often loses the last 4 bytes of data. This is a
[known issue](https://www.esp32.com/viewtopic.php?t=7339). Even
though the test harness transfera 16 bytes, only 12 are expected to be
returned.

In addition to SPI, the test harness uses the following
GPIO pins:

| Pin No. | I/O | Description
| ------- | --- | -------------------------------------------------- |
|       2 | Out | On-board LED. Displays error codes & debug info. Reserved, not used. |
|       4 | In  | Slave Ready Not. Low --> slave is up and running. |
|      16 | In  | SPI Clear To Send Not. Low --> master can send data. |
|      17 | Out | RoboRio Wants Data Not. The test harness lowers this pin to signal that it wants data. Currently unused. |
|      26 | Out | RboRio Wants Data Fwd Relay, simulates a FWD relay signal. Compatible with an Opto-Isolator; prefer to RoboRio Wants Data Not. |
|      12 | Out | Green LED |
|      14 | Out | Yellow LED |
|      27 | Out | Green LED |

## Program Logic

The program logic is straightforward. The `setUp` function configures GPIO pins and sets
output pins to their initial values. It then initializes the SPI bus and configures the
inclinometer as a slave device. If initialization failes, the program enters a blink
loop that displays a diagnostic code.

The test harness pauses for 5 seconds after initialization completes
to give the inclinometer time to configure itself.

The test harness queries the inclinometer every 5 seconds. The test sequence is:

1. Zero fill the receive buffer and fill the transmit buffer with test data.
2. Lower RoboRio Wants Data Not (currently unused). Raise RoboRio Wants Data Fwd Relay
   to simulate the RoboRio FWD relay signal.
3. Acquire the SPI bus. Strictly speaking, this is not required, but
   it is good practice.
4. Wait for the Inclinometer to lower SPI Clear To Send Not. Since this
   should happen within a few microseconds, this is done with a spin
   loop.
5. Exchange data with the inclinometer.
6. Wait for the inclinometer to raise SPI Clearn To Send Not. This
   indicates that the inclinometer can accept another request.
7. Convert the raw received data into the angle in tenths of a degree
   and display both the raw and converted values.

Note that the test harness illuminates LEDs to indicate its
status. This is essential for debugging.


