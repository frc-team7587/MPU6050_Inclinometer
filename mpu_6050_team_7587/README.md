# Team 7587 MPU 6050-Based Inclinometer

## Status

| Attribute | State |
| --------- | ----- |
| Documentation | Preliminary Draft |
| Software | Proof of Concept |
| Hardware Design | Prototype |
| Packaging | Fragile Breadboard |

## Background

[FIRST Energize](https://info.firstinspires.org/first-energize-season?utm_source=first-inspires&utm_medium=about-frc&utm_campaign=fir-registration-023),
the 2023 [FIRST Robotics](https://www.firstinspires.org/)
challenge, requires robots to balance on a platform. To meet the challenge,
Team 7587, [Metuchen Momentum](https://www.metuchenmomentum.org/) has developed
a prototype [MPU-6050](https://www.albany.edu/faculty/dsaha/teach/2020Fall_ECE553/resources/MPU-6000-Datasheet1.pdf)-based
inclinometer that determines the robot's pitch. The team will use the pitch
angle to control the robot when it is balancing on the platform.

## Overview

The MPU 6050-Based Inclinometer is a hardware and software package that continuously
monitors pitch and roll angles, and sends the current pitch angle to the RoboRio on
command. The inclinometer uses the
[Serial Peripheral Interface (SPI)](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface)
protocol and DIO to communicate with the RoboRio.

An ESP32s microcontroller](https://www.espressif.com/en/products/socs/esp32) manages
the MPU-6050, updating it at 2350 MHz and sending readings to the RoboRio on command.
The RoboRio must be connected to the ESP32's `VSPI` (aka `SPI3_HOST`) bus. 

## Communications

As indicated above, the inclinometer communicates with the RoboRio via the latter's
[Serial Peripheral Interface (SPI) bus](https://docs.wpilib.org/en/stable/docs/hardware/sensors/serial-buses.html#spi).
The ESP32's `VSPI` pins must be connected to their counterparts on the RoboRio.Connect the ESP32's Slave Select pin
to the RoboRio's lowest numbered available CS pins. Consult the accompanying pinout diagram and the
`VSPIPinAssignments.h` file for pin assignments.

In addition to SPI, the inclinometer provides two 4N35 Optocouplers that allows the ESP32, a 3.3 Volt
device, to communicate safely with the RoboRio's 5 Volt pins. The isolator's 5 Volt
circult uses an NPN transistor which must be connected to a DIO pin **exactly** as described.
Be extremely careful. **REVERSING THE CONNECTIONS WILL RUIN THE OPTOIOLATOR.**" Don't be *that* member!
Consult the [DataSheet](https://www.vishay.com/docs/81181/4n35.pdf), and have someone inspect your
work before you apply power.

The first optocoupler is connected to the DIO pin that the ESP32 uses to signal that
data is available.
Connect the optocoupler **exactly** as specified below.

| 4N35 Pin | Transitor Junction | DIO Pin |
| -------- | ------------------ | ------- |
|        4 | Base               | Ground  |
|        5 | Collector          | S (signal) |

The second optocoupler is connected to the relay pin that the RoboRio uses to request data.
Connect the optocoupler **exactly** as specified below.

| 4N35 Pin | Diode Junction | DIO Pin |
| -------- | -------------- | ------- |
|        1 | Anode (+)      | FWD     |
|        2 | Cathode (-)    | Ground  |


Note that pins 1 - 3 are reserved for the ESP32, and that pin 6 is connected to the
transistor's base. **DO NOT CONNECT PIN 6 TO ANYTHING.**

## Data Transfer

As stated above, the inclinometer transmits data to the RoboRio over a full duplex
SPI connection. The transfer must be configured as follows.

| Setting | Value |
| ------- | ----- |
| SPI Mode | 0 (Zero) |
| Buffer Length | 16 bytes (see note) |
| Speed | 1 MHz max, 1 MHz recommended |

Note: the ESP32 SPI often loses the last 4 bytes of data. This is a
[known issue](https://www.esp32.com/viewtopic.php?t=7339). Even
though the RoboRio **must** transfer 16 bytes, only 12 will be
returned.

Data is returned in an array of 16 8 bit bytes. The closest
Java type is the `byte`. The returned data is the inclination in
tenths of a degree, formatted in hexidecimal encoded from a
16 bit, two's compliement integer corresponding to Java's
`short` type. The buffer is zero-filled on the right, and
there are no leading 0s. 

Note that the byte buffer is **not** a `String` nor an array of `char`.
It is an array of raw, 8 bit ASCII characters whose values are guaranteed
to be between `0x20` and '0xFE' (`' '` and `'~'`) inclusive. Therefore,
it should be possible to convert them into a `String`. It is strongly
recommended that the code construct the `String` frin the non-zero portion of the
array *only*.

Communication takes place as follows:

1. The RoboRio requests data by energizing the FWD pin on the
   request data relay port. This sends current through the relay
   pin optocoupler which drives the ESP32's `ROBO_RIO_WANTS_DATA_NOT`
   pin low and starts the data transfer.
2. The RoboRio waits on the DIO pin for data to become ready.
3. Meanwhile, the ESP32 determines the current pitch reading in tenths of a
   degree, converts the value into hexadecimal, and enqueues an
   SPI message containing the converted value.
3. The ESP32 applies current to the DIO optocoupler, which
   allows current to flow from the DIO signal pin to ground.
4. The RoboRio senses that data is available and performs the
   SPI transfer.
5. The RoboRio waits for the ESP32 to deenergize the DIO
   optocoupler and waits for `ROBO_RIO_WANTS_DATA_NOT` to
   go high.
6. The ESP32 senses the end of the SPI communication and
   deenergizes the DIO optocoupler. This interrupts the
   current flow from signal to ground, which the RoboRio
   interprets as switch open.
7. The RoboRio senses that the DIO switch is open, and de-energizes
   the FWD pin on the request data relay port. This forces the
   ESP32's `ROBO_RIO_WANTS_DATA_NOT` pin high.
8. The ESP32 senses that `ROBO_RIO_WANTS_DATA_NOT` has gone high
   and makes itself ready to process the next request.

## Tasks Remaining

The MCU-6050 has stability problems. It's pitch measurements are
probably accurate and stable enough for use in a pinch, but we should
try other gyros. The team has obained a higher quality unit to evaluate.

That said, the following needs to be done to productionize this
hardware/software configuration.

1. The hardware needs to be packaged for competetion. 
2. The ESP32 code that manages the handshake and transmission should
   be moved to a separate task that receives the angle via a queue. This
   takes the response code out of the gyro update loop, which has a strict
   deadline. This is a relatively straightforward refactoring that should have
   no user-visible effects, except for providing more accurate readings.
3. We should consider providing a way for the RoboRio to reset the ESP32
   should things go horribly wrong.

## Bibliography

* [MPU-6050 Datasheet](https://www.albany.edu/faculty/dsaha/teach/2020Fall_ECE553/resources/MPU-6000-Datasheet1.pdf)
* [4N35 Optocoupler Datasheet](https://www.vishay.com/docs/81181/4n35.pdf)
* [ESP32 Product Overview](https://www.espressif.com/en/products/socs/esp32)
* [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html). Includes
  getting started, hardware API, and FreeRTOS APIs.
* [Based On](https://dronebotworkshop.com/mpu-6050-level/)
* [RoboRio SPI Bus](https://docs.wpilib.org/en/stable/docs/hardware/sensors/serial-buses.html#spi)


