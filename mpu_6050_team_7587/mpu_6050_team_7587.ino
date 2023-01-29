/**
 * Electronic Level
 * mpu-6050-level.ino
 * Uses MPU-6050 IMU
 * Displays on 128x64 OLED and LED
 *
 * DroneBot Workshop 2019
 * https://dronebotworkshop.com
 *
 * Shamelessly cribbed from: https://dronebotworkshop.com/mpu-6050-level/
 *
 * See PinAssignments.h for pin use.
 */

#include "Arduino.h"
#include <stdlib.h>

#include "driver/spi_slave.h"

#include <Wire.h>

#include <LiquidCrystal_I2C.h>
#include "VSPIslave.h"

#include "PinAssignments.h"
#include "VSPIPinAssignments.h"

#define LIQUID_CRYSTAL_DISPLAY_I2C_ADDRESS 0x27

/**
 * The liquid crystal display (LCD) that shows initialization status on
 * startup, and pitch and roll when running.
 */
LiquidCrystal_I2C lcd(
    LIQUID_CRYSTAL_DISPLAY_I2C_ADDRESS, // Address on the I2C bus
    16, // Display column count
    2); // Display row count

/**
 * SPI configuration for exchanging data with the RoboRio
 */
VSPI_slave spi_slave(
    GPIO_SS,
    SPI_CLEAR_TO_SEND_NOT,
    SPI_CLEAR_TO_SEND_SWITCH,
    SPI_MODE0);

/**
 * We use a 16 byte buffer. Note that we can only count on 12 bytes being sent.
 * This is a known issue. See
 */
#define SPI_BUFFER_LENGTH 16

uint8_t alternating_leds[] = {
  FRONT_RED_LED,
  BACK_RED_LED,
  FRONT_YELLOW_LED,
  BACK_YELLOW_LED,
  FRONT_GREEN_LED,
  BACK_GREEN_LED,
  BLUE_LED,
};

#define NUMBER_OF_LEDS (sizeof(alternating_leds))

struct PinAndLevel {
  uint8_t pin;
  uint8_t level;
};

PinAndLevel led_illumination[] = {
  {FRONT_RED_LED, LOW},
  {FRONT_YELLOW_LED, LOW},
  {FRONT_GREEN_LED, LOW},
  {BLUE_LED, LOW},
  {BACK_GREEN_LED, LOW},
  {BACK_YELLOW_LED, LOW},
  {BACK_RED_LED, LOW},
};

DMA_ATTR WORD_ALIGNED_ATTR uint8_t spi_send_buffer[SPI_BUFFER_LENGTH];
DMA_ATTR WORD_ALIGNED_ATTR uint8_t spi_receive_buffer[SPI_BUFFER_LENGTH];

// To eliminate any possible incompatibilities such as endian or floating
// point formats, pitch values are converted to signed, 16-bit integer
// values in tenthes of a degree. In principal, values can range from -1800 to
// +1800, but should not exceed +/- 200 (that is 20 degrees) in practice.
// Integers are converted into four digit hexadecimal ASCII characters.
// Integer to hexadecimal conversion uses masking, shifting, and table
// lookup, while converting to decimal requires division, which is far slower.
// The resulting ASCII characters are unsigned, 8-bit integers. The
// corresponding Java type is the byte, which is, alas, a signed 8-bit integer.
// Since ASCII values are limited to [0x00 .. 0xFF] ([0 .. 127] decimal), this
// does not pose a problem.
//
// Note:
//
//  1. The byte array is null-terminated.
//  2. The 16 bit integer corresponds exactly to Java's short type. The
//     receiver can recover the integer value by masking, oring, and shifting.
//
//

//Variables for Gyroscope
int16_t gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
bool set_gyro_angles;

int16_t acc_x, acc_y, acc_z;

long acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

// Setup timers and temp variables
unsigned long loop_timer;
int temp;

// Display counter
int displaycount = 0;

bool halted = false;

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

void set_all_leds(uint8_t mode) {
  digitalWrite(FRONT_RED_LED, mode);
  digitalWrite(FRONT_YELLOW_LED, mode);
  digitalWrite(FRONT_GREEN_LED, mode);
  digitalWrite(BLUE_LED, mode);
  digitalWrite(BACK_GREEN_LED, mode);
  digitalWrite(BACK_YELLOW_LED, mode);
  digitalWrite(BACK_RED_LED, mode);
}

void setup() {
  Serial.begin(115200);
  Serial.print("Team 7587 MPU6050 Inclinometer compiled on ");
  Serial.print(__DATE__);
  Serial.print(" at ");
  Serial.println(__TIME__);
  Serial.print("Initial angle values -- pitch: ");
  Serial.print(angle_pitch_output);
  Serial.print(", roll: ");
  Serial.println(angle_roll_output);

  //Start I2C
  Wire.begin();

  //Setup the registers of the MPU-6050
  setup_mpu_6050_registers();
  Serial.println("MPU6050 registers initialized.");

  // Display initialization message
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing ...");
  lcd.setCursor(0, 1);

  // Set Level LEDs as outputs
  pinMode(FRONT_RED_LED, OUTPUT);
  pinMode(FRONT_YELLOW_LED, OUTPUT);
  pinMode(FRONT_GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BACK_GREEN_LED, OUTPUT);
  pinMode(BACK_YELLOW_LED, OUTPUT);
  pinMode(BACK_RED_LED, OUTPUT);

  pinMode(MOSI, INPUT_PULLUP);
  pinMode(MISO, OUTPUT);
  pinMode(SCK, INPUT_PULLUP);
  pinMode(SS, INPUT_PULLUP);
  pinMode(SPI_CLEAR_TO_SEND_NOT, OUTPUT);
  pinMode(SPI_CLEAR_TO_SEND_SWITCH, OUTPUT);
  pinMode(SLAVE_READY_NOT, OUTPUT);

  pinMode(EMERGENCY_STOP_PIN, INPUT);
  pinMode(ROBO_RIO_WANTS_DATA_NOT, INPUT_PULLUP);
  digitalWrite(SLAVE_READY_NOT, HIGH);
  // Definitely not ready for SPI input.
  digitalWrite(SPI_CLEAR_TO_SEND_NOT, HIGH);
  digitalWrite(SPI_CLEAR_TO_SEND_SWITCH, LOW);

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  Serial.println("Initializing SPI.");
  if (spi_slave.begin() != ESP_OK) {
    error_loop(1);
  }
  Serial.println("SPI initialized successfully.");

  pinMode(INDICATOR_LED, OUTPUT);
  for (int i = 0; i < 4; ++i) {
    digitalWrite(INDICATOR_LED, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(INDICATOR_LED, LOW);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  digitalWrite(INDICATOR_LED, HIGH);

  // LED Test
  set_all_leds(LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  for (uint8_t led_index = 0; led_index < sizeof(alternating_leds); ++led_index) {
    digitalWrite(alternating_leds[led_index], HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  set_all_leds(LOW);
  digitalWrite(INDICATOR_LED, LOW);
  vTaskDelay(pdMS_TO_TICKS(1000));

  //Read the raw acc and gyro data from the MPU-6050 1000 times
  uint8_t led_mode = LOW;
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){
    read_mpu_6050_data();
    //Add the gyro x offset to the gyro_x_cal variable
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable
    gyro_y_cal += gyro_y;
    //Add the gyro z offset to the gyro_z_cal variable
    gyro_z_cal += gyro_z;
    //Delay 3us to have 250Hz for-loop
    delay(3);
    if (cal_int % 50 == 1) {
      led_mode = (led_mode == LOW) ? HIGH : LOW;
      set_all_leds(led_mode);
    }
    if ((cal_int % 100) == 1) {
      lcd.print('.');
    }
  }
  set_all_leds(LOW);
  digitalWrite(INDICATOR_LED, LOW);

  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

  Serial.print("Calibration -- x: ");
  Serial.print(gyro_x_cal);
  Serial.print(", y: ");
  Serial.print(gyro_y_cal);
  Serial.print(", z: ");
  Serial.println(gyro_z_cal);

  lcd.clear();

  digitalWrite(SLAVE_READY_NOT, LOW);

  // Init Timer. This must be the last statement in setUp to ensure that the
  // first loop runs for exactly 4000 microseconds, which is 4 milliseconds.
  // We measure time in microseconds to reduce clock jitter to to the absolute
  // minimum of +/- .5 microseconds. If we used millisecond resolution, jitter
  // would be 1000 times higher at +/- .5 milliseconds, or +/- 500 microseconds.
  // We set the LEDs and write the values to the LCD display every 100 cycles,
  // that is every 400 milliseconds. Response time to the RoboRio is at
  // most 4000 microseconds (4 milliseconds).
  //
  // We check for a read request at loop start and assume that the RoboRio
  // will finish reading quickly enough for us to complete the current
  // iteration within 4000 ms. Since transmission should complete in about
  // two microseconds at a conservative 20MHz, this should not pose a problem.
  loop_timer = micros();
}

void loop(){

  halted = halted || digitalRead(EMERGENCY_STOP_PIN) == LOW;

  if (halted) {
    lcd.clear();
    lcd.print("Halted ...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  }

  // If the RoboRio wants data, send it!
  if (digitalRead(ROBO_RIO_WANTS_DATA_NOT) == LOW) {
    memset(spi_receive_buffer, 0, sizeof(spi_receive_buffer));
    memset(spi_send_buffer, 0, sizeof(spi_send_buffer));
    int16_t angle_in_tenths_of_a_degree = (angle_pitch * 10);
    itoa(angle_in_tenths_of_a_degree, (char *) spi_send_buffer, HEX);
    if (spi_slave.send(
        spi_send_buffer,
        spi_receive_buffer,
        SPI_BUFFER_LENGTH)
        != ESP_OK) {
      error_loop(2);
    }
    while(digitalRead(ROBO_RIO_WANTS_DATA_NOT)) {
      vTaskDelay(1);
    }
  }

  // Get data from MPU-6050
  read_mpu_6050_data();

  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)

  //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_x * 0.0000611;
  //Calculate the traveled roll angle and add this to the angle_roll variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_roll += gyro_y * 0.0000611;

  //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  //If the IMU has yawed transfer the pitch angle to the roll angle
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);

  //Accelerometer angle calculations

  //Calculate the total accelerometer vector
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));

  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
  //Calculate the roll angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;

  //Accelerometer calibration value for pitch
  angle_pitch_acc -= 0.0;
  //Accelerometer calibration value for roll
  angle_roll_acc -= 0.0;

  if(set_gyro_angles){
    //If the IMU has been running
    //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    //Correct the drift of the gyro roll angle with the accelerometer roll angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  } else{
    //IMU has just started
    //Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_pitch = angle_pitch_acc;
    //Set the gyro roll angle equal to the accelerometer roll angle
    angle_roll = angle_roll_acc;
    //Set the IMU started flag
    set_gyro_angles = true;
  }

  // To dampen the pitch and roll angles a complementary filter is used
  // Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  //Take 90% of the output roll value and add 10% of the raw roll value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;


  // Increment the display counter
  displaycount = displaycount +1;

  if (displaycount > 100) {

    lcd.clear();
    // Print on first row of LCD
    lcd.setCursor(0,0);
    lcd.print("Pitch: ");
    lcd.print(angle_pitch_output);
    lcd.setCursor(0,1);
    lcd.print("Roll: ");
    lcd.print(angle_roll_output);

    // Check Angle for Level LEDs

    for (int led_no = 0; led_no < NUMBER_OF_LEDS; ++led_no) {
      led_illumination[led_no].level = LOW;
    }

    if (angle_pitch_output < -10.0) {
      led_illumination[0].level = HIGH;
    } else if (angle_pitch_output < -6) {
      led_illumination[1].level = HIGH;
    } else if (angle_pitch_output < -2) {
      led_illumination[2].level = HIGH;
    } else if (angle_pitch_output < 2) {
      led_illumination[3].level = HIGH;
    } else if (angle_pitch_output < 6) {
      led_illumination[4].level = HIGH;
    } else if (angle_pitch_output < 10){
      led_illumination[5].level = HIGH;
    } else {
      led_illumination[6].level = HIGH;
    }

    PinAndLevel *led_to_illuminate = led_illumination;
    for (int led_no = 0; led_no < NUMBER_OF_LEDS; ++led_no) {
      digitalWrite(led_to_illuminate->pin, led_to_illuminate->level);
      ++led_to_illuminate;
    }

  displaycount = 0;

  }

  // Wait until the loop_timer reaches 4000us (250Hz) before starting the
  // next loop.
  //
  // Spin until 4000 microseconds have elapsed since the last iteration,
  // finished. Note that the measurement includes the system overhead needed
  // to restart the loop, and that the spin loop is so short that it does
  // not trigger the FreeRTOS watchdog timer.
  while(micros() - loop_timer < (unsigned long) 4000);

  // Set the start time for the next iteration. By setting the start time
  // here, we ensure that the 4000 microsecond time includes the overhead
  // required to restart the loop.
  loop_timer = micros();
}

void setup_mpu_6050_registers(){

  Serial.println("Configuring the MPU6050");
  //Activate the MPU-6050

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x6B);
  //Set the requested starting register
  Wire.write(0x00);
  //End the transmission
  Wire.endTransmission();
  Serial.println("MPU6050 activated.");


  //Configure the accelerometer (+/-8g)

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x1C);
  //Set the requested starting register
  Wire.write(0x10);
  //End the transmission
  Wire.endTransmission();
  Serial.println("Accelerometer configured.");

  //Configure the gyro (500dps full scale)

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x1B);
  //Set the requested starting register
  Wire.write(0x08);
  //End the transmission
  Wire.endTransmission();
  Serial.println("Gyro configured.");

}


void read_mpu_6050_data(){

  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x3B);
  //End the transmission
  Wire.endTransmission();
  //Request 14 bytes from the MPU-6050
  Wire.requestFrom(0x68,14);
  //Wait until all the bytes are received
  while(Wire.available() < 14);

  //Following statements left shift 8 bits, then bitwise OR.
  //Turns two 8-bit values into one 16-bit value
  acc_x = Wire.read()<<8|Wire.read();
  acc_y = Wire.read()<<8|Wire.read();
  acc_z = Wire.read()<<8|Wire.read();
  temp = Wire.read()<<8|Wire.read();
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();
}
