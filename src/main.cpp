#include <Arduino.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SD.h>
#include <Metro.h>
#include <TimeLib.h>

int Steering = 14;
int SteeringVal;
File logger;
uint64_t global_ms_offset = 0;
Metro timer_flush = Metro(50);
uint64_t last_sec_epoch;
Metro timer_debug_RTC = Metro(1000);
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// #define LSM9DS1_SCL = 19
// #define LSM9DS1_SDA = 18

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void sd_date_time(uint16_t *date, uint16_t *time)
{
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(), month(), day());
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(), minute(), second());
}

void setupSD()
{

  // Set up real-time clock
  // Teensy3Clock.set(1660351622); // set time (epoch) at powerup  (COMMENT OUT THIS LINE AND PUSH ONCE RTC HAS BEEN SET!!!!)
  setSyncProvider(getTeensy3Time); // registers Teensy RTC as system time
  if (timeStatus() != timeSet)
  {
    Serial.println("RTC not set up - uncomment the Teensy3Clock.set() function call to set the time");
  }
  else
  {
    Serial.println("System time set to RTC");
  }
  last_sec_epoch = Teensy3Clock.get();

  Serial.println("Initializing SD card...");
  SdFile::dateTimeCallback(sd_date_time); // Set date/time callback function
  // SD.begin(BUILTIN_SDCARD); do no oneed this line, it is auto ran in one of the header files
  if (!SD.begin(BUILTIN_SDCARD))
  { // Begin Arduino SD API (Teensy 3.5)
    Serial.println("SD card failed or not present");
  }

  char filename[] = "data0000.CSV";
  for (uint8_t i = 0; i < 10000; i++)
  {
    filename[4] = i / 1000 + '0';
    filename[5] = i / 100 % 10 + '0';
    filename[6] = i / 10 % 10 + '0';
    filename[7] = i % 10 + '0';
    if (!SD.exists(filename))
    {
      Serial.println("Writing SD card File Name");
      logger = SD.open(filename, (uint8_t)O_WRITE | (uint8_t)O_CREAT); // Open file for writing
      break;
    }
    if (i == 9999)
    { // If all possible filenames are in use, print error
      Serial.println("All possible SD card log filenames are in use - please clean up the SD card");
    }
  }
}

void write_to_SD()
{ // Note: This function does not flush data to disk! It will happen when the buffer fills or when the above flush timer fires
  // Calculate Time

  // This block is verified to loop through

  uint64_t sec_epoch = Teensy3Clock.get();
  if (sec_epoch != last_sec_epoch)
  {
    global_ms_offset = millis() % 1000;
    last_sec_epoch = sec_epoch;
  }
  uint64_t current_time = sec_epoch * 1000 + (millis() - global_ms_offset) % 1000;

  // Log to SD
  logger.print(current_time);
  logger.print(",high");
  // logger.print(msg->id, HEX);
  logger.print(",");
  // logger.print(msg->len);
  logger.print(",");
  /*
  for (int i = 0; i < msg->len; i++) {
      if (msg->buf[i] < 16) {
          logger.print("0");
      }
      logger.print(msg->buf[i], HEX);
  }*/
  logger.println();
}

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Steering, INPUT);
  Serial.begin(9600);
  /*
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  */
  Serial.println("LSM9DS1 data read test");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1)
      ;
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  setupSD();
}

// the loop function runs over and over again forever
void loop()
{

  SteeringVal = analogRead(Steering);
  Serial.println(SteeringVal);
  delay(50);

  lsm.read(); /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);
  /*
    Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
    Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
    Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

    Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
    Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
    Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

    Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" rad/s");
    Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" rad/s");
    Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" rad/s");
  */
  Serial.println("running");
  delay(200);
  write_to_SD();

  if (timer_flush.check())
  {
    logger.flush(); // Flush data to disk (data is also flushed whenever the 512 Byte buffer fills up, but this call ensures we don't lose more than a second of data when the car turns off)
  }
  /* Print timestamp to serial occasionally */
  if (timer_debug_RTC.check())
  {
    Serial.println(Teensy3Clock.get());
    // msg_tx.id=0x3FF;
    // CAN.write(msg_tx);
  }

  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  // delay(500);                      // wait for a second
  // digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  // delay(500);                      // wait for a second
}
