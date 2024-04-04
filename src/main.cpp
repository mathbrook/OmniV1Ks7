#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>
#include <SparkFunLSM9DS1.h>
#include <Metro.h>
#include <omnican.h>
#include <analogsensor.h>
#include <accelerometer.h>
#include <pindefs.h>

#define SHOCKPOT_CUTOFF_FREQUENCY 10
#define STEERING_CUTOFF_FREQUENCY 20

// Sensors
analogSensor fl_shockpot = analogSensor(analogRead, flpin, SHOCKPOT_CUTOFF_FREQUENCY);
analogSensor fr_shockpot = analogSensor(analogRead, frpin, SHOCKPOT_CUTOFF_FREQUENCY);
analogSensor rl_shockpot = analogSensor(analogRead, rlpin, SHOCKPOT_CUTOFF_FREQUENCY);
analogSensor rr_shockpot = analogSensor(analogRead, rrpin, SHOCKPOT_CUTOFF_FREQUENCY);
analogSensor steering_angle = analogSensor(analogRead, steeringpin, STEERING_CUTOFF_FREQUENCY);

analogSensor *sensors[] = {
    &fl_shockpot,
    &fr_shockpot,
    &rl_shockpot,
    &rr_shockpot,
    &steering_angle};

accelerometer imu;
// End of sensors

File logger;
uint64_t global_ms_offset = 0;
Metro timer_flush = Metro(50);
uint64_t last_sec_epoch;
Metro timer_debug_RTC = Metro(1000);

Metro timer_heartbeat = Metro(1000);
Metro timer_print = Metro(250);
#define LOGGING_FREQUENCY 100                            // Hz, Frequency to log to SD
#define LOGGING_PERIOD 1000/LOGGING_FREQUENCY
Metro timer_WriteToSD = Metro(1000 / LOGGING_FREQUENCY); // Logging to SD timer. Period = 1000ms / LOGGING_FREQUENCY
Metro timer_StatusLEDon = Metro(1000);                   // time for status led to stay on

//
unsigned long loop_count = 0; // Track the number of loop executions between writes to estimate sample coutn/freq
elapsedMillis loop_duration;  // Reset at the start of each loop
uint16_t samples_per_sec = 0;
//
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
  // READ: Manual setting of the clock is not needed on teensy. The Teensy loader .exe will always update the RTC when you flash.
  //       Just make sure to observe that the RTC has updated correctly whenever you are handling it
  // Teensy3Clock.set(1692325055); // set time (epoch) at powerup  (COMMENT OUT THIS LINE AND PUSH ONCE RTC HAS BEEN SET!!!!)
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
  while (!SD.begin(BUILTIN_SDCARD))
  {
    static uint8_t trycoutn = 5;
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
    digitalWrite(LED_RED, HIGH);
    if (timer_print.check())
    {
      Serial.println("SD card failed or not present");
      Serial.println("Retrying...");
      trycoutn--;
    }
    if (trycoutn <= 0)
    {
      break;
    }
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
  // Set header of .csv
  logger.print("Time,Steering,ShockFL,ShockFR,ShockRL,ShockRR,Accel-X,Accel-Y,Accel-Z,Heading,Pitch,Roll,Gyro-X,Gyro-Y,Gyro-Z\n");
}

void write_to_SD()
{ // Note: This function does not flush data to disk! It will happen when the buffer fills or when the above flush timer fires

  uint64_t sec_epoch = Teensy3Clock.get();
  if (sec_epoch != last_sec_epoch)
  {
    global_ms_offset = millis() % 1000;
    last_sec_epoch = sec_epoch;
  }
  uint64_t current_time = sec_epoch * 1000 + (millis() - global_ms_offset) % 1000;

  logger.printf("%d,%d,%d,%d,%d,%d", current_time, steering_angle.getValue(), fl_shockpot.getValue(), fr_shockpot.getValue(), rl_shockpot.getValue(), rr_shockpot.getValue());
  logger.printf(",%d,%d,%d", imu.accelData.x, imu.accelData.y, imu.accelData.z);
  logger.printf(",%d,%d,%d", imu.attitudeData.x,imu.attitudeData.y,imu.attitudeData.z);
  logger.printf(",%d,%d,%d",imu.gyroData.x,imu.gyroData.y,imu.gyroData.z);
  logger.println();

  // Log to SD
  // TODO replace with CAN logging stuff
}

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize GPIOs
  init_inputs(pinMode, analogReadRes, adcRes::ADC_12BIT);
  init_outputs(pinMode);

  can_setup(500000);

  // Start accelerometer I2C
  Wire.begin();

  // Try to initialise and warn if we couldn't detect the chip
  if (!imu.init())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    // while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSD();
  digitalWrite(LED_RED, LOW);
  loop_duration = 0;
}
// the loop function runs over and over again forever
void loop()
{

  digitalWrite(LED_BLUE, LOW);

  // Run IMU functions
  imu.run();

  // Fetch ADC reads
  for (uint8_t i=0; i < sizeof(sensors) / sizeof(sensors[0]); i++)
  {
    sensors[i]->run();
  }

  Serial.printf("STEERING: %d FL: %d FR: %d RL: %d RR: %d\n", steering_angle.getValue(), fl_shockpot.getValue(), fr_shockpot.getValue(), rl_shockpot.getValue(), rr_shockpot.getValue());

  Serial.printf("Ax: %d Ay: %d Az: %d", imu.accelData.x, imu.accelData.y, imu.accelData.z);

  // Serial.printf("%f,%f,%f\n", imu.accelData.x,imu.accelData.y,imu.accelData.z);
  if (timer_print.check())
  {
    // Print analog readings
    // Serial.printf("FL: %d FR: %d RL: %d RR: %d\n",fl_shockpot.getValue(),fr_shockpot.getValue(),rl_shockpot.getValue(),rr_shockpot.getValue());
    // // Print accelerometer readings:
    // Serial.printf("Ax: %d Ay: %d Az: %d",imu.accelData.x,imu.accelData.y,imu.accelData.z);
  }

  if (timer_WriteToSD.check())
  {
    samples_per_sec = loop_count / (loop_duration / 1000);
    digitalWrite(LED_BLUE, HIGH);
    write_to_SD();
    loop_duration = 0;
    loop_count = 0;
  }

  if (timer_flush.check())
  {
    logger.flush(); // Flush data to disk (data is also flushed whenever the 512 Byte buffer fills up, but this call ensures we don't lose more than a second of data when the car turns off)
  }

  // TODO: Make this match what the LoRa does
  if (timer_debug_RTC.check())
  {
    // CAN_message_t msg_tx;
    Serial.println(Teensy3Clock.get());
    // msg_tx.id=0x3FF;
    // Can0.write(msg_tx);
  }

  if (timer_heartbeat.check())
  {
    // Toggle built-in LED to show we are alive
    digitalToggle(LED_BUILTIN);
  }
}