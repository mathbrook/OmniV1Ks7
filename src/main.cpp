#include <Arduino.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SD.h>
#include <Metro.h>
#include <TimeLib.h>
#include <string>
#include <SimpleKalmanFilter.h>
int Steering = 21;
int SteeringVal;

int FRShock = 17;
int FLShock = 16;
int RRShock = 15;
int RLShock = 14;
int FRShockVal;
int FLShockVal;
int RRShockVal;
int RLShockVal;

int LED_RED = 6;
int LED_GREEN = 7;
int LED_BLUE = 8;


const float cutoffhz = 10; // Hz, exponential filter low pass cutoff
// Calculate filtering alpha value for the cutoff frequency  
const double FILTERING_ALPHA = 2 * 3.14 * cutoffhz / (1 + 2 * 3.14 * cutoffhz);

File logger;
uint64_t global_ms_offset = 0;
Metro timer_flush = Metro(50);
uint64_t last_sec_epoch;
Metro timer_debug_RTC = Metro(1000);
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

Metro timer_heartbeat = Metro(1000);
Metro timer_print = Metro(250);
#define LOGGING_FREQUENCY 100 // Hz, Frequency to log to SD
Metro timer_WriteToSD = Metro(1000 / LOGGING_FREQUENCY); // Logging to SD timer. Period = 1000ms / LOGGING_FREQUENCY
Metro timer_StatusLEDon = Metro(1000); // time for status led to stay on

// #define LSM9DS1_SCL = 19
// #define LSM9DS1_SDA = 18

SimpleKalmanFilter accelX(1,1,0.01); //accelerometer X
SimpleKalmanFilter accelY(1,1,0.01); //accelerometer Y
SimpleKalmanFilter accelZ(1,1,0.01); //accelerometer Z
float ax,ay,az;
//
SimpleKalmanFilter gyroX(1,1,0.01); //gyro roll
SimpleKalmanFilter gyroY(1,1,0.01); //gyro pitch
SimpleKalmanFilter gyroZ(1,1,0.01); //gyro heading
float gx,gy,gz;

String Accelx;
String Accely;
String Accelz;
String filt_ax;
String filt_ay;
String filt_az;
String Magx;
String Magy;
String Magz;
String Gyrox;
String Gyroy;
String Gyroz;
String filt_gx;
String filt_gy;
String filt_gz;
String SteeringOut;
String FRShockOut;
String FLShockOut;
String RLShockOut;
String RRShockOut;

unsigned long loop_count = 0; // Track the number of loop executions between writes to estimate sample coutn/freq
elapsedMillis loop_duration; // Reset at the start of each loop
uint16_t samples_per_sec = 0;

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
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
    digitalWrite(LED_RED, HIGH);
    // if (!SD.begin(BUILTIN_SDCARD))
    // { // Begin Arduino SD API (Teensy 3.5)
    if (timer_print.check())
    {
      Serial.println("SD card failed or not present");
      Serial.println("Retrying...");
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
  logger.print("Time,Output,Steering Value, Accel-X, Accel-Y, Accel-Z, Mag-X, Mag-Y, Mag-Z, Gyro-X, Gyro-Y, Gyro-Z, ShockFL, ShockFR, ShockRL, ShockRR, SampleCount,filtax,filtay,filtaz");
  logger.println();
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
  logger.print("," + SteeringOut);
  // logger.print(msg->len);
  logger.print("," + Accelx);
  logger.print("," + Accely);
  logger.print("," + Accelz);
  logger.print("," + Magx);
  logger.print("," + Magy);
  logger.print("," + Magz);
  logger.print("," + Gyrox);
  logger.print("," + Gyroy);
  logger.print("," + Gyroz);
  logger.print("," + FLShockOut);
  logger.print("," + FRShockOut);
  logger.print("," + RLShockOut);
  logger.print("," + RRShockOut);
  logger.printf("%d,%f,%f,%f,%f,%f,%f",samples_per_sec,ax,ay,az,gx,gy,gz);
  logger.println();
}
void update_value_filtered(int &old_reading,int new_reading, const double ADC_ALPHA);

// the setup function runs once when you press reset or power the board
void setup()
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Steering, INPUT);
  pinMode(FRShock, INPUT);
  pinMode(FLShock, INPUT);
  pinMode(RRShock, INPUT);
  pinMode(RLShock, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
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
    // while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  setupSD();
  loop_duration = 0;
}
// the loop function runs over and over again forever
void loop()
{
  digitalWrite(LED_BLUE, LOW);
  
  SteeringVal = analogRead(Steering)-657; //-657 from sensor installed on 8/17/23 and calibrated
  update_value_filtered(FRShockVal,analogRead(FRShock),FILTERING_ALPHA);
  update_value_filtered(FLShockVal,analogRead(FLShock),FILTERING_ALPHA);
  update_value_filtered(RRShockVal,analogRead(RRShock),FILTERING_ALPHA);
  update_value_filtered(RLShockVal,analogRead(RLShock),FILTERING_ALPHA);

  lsm.read(); /* ask it to read in the data */

  /* Get a new sensor event */

  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);
  ax = accelX.updateEstimate(a.acceleration.x);
  ay = accelY.updateEstimate(a.acceleration.y);
  az = accelZ.updateEstimate(a.acceleration.z);

  gx = gyroX.updateEstimate(g.gyro.x);
  gy = gyroY.updateEstimate(g.gyro.y);
  gz = gyroZ.updateEstimate(g.gyro.z);

  digitalWrite(LED_RED, LOW);

  if (timer_StatusLEDon.check())
  {
    digitalWrite(LED_BLUE, LOW);
  }

  if (timer_print.check())
  {
    Serial.print("Steering Val: ");
    Serial.println(SteeringVal);

    Serial.print("Front Right shock value: ");
    Serial.println(FRShockVal);
    Serial.print("Front Left shock value: ");
    Serial.println(FLShockVal);
    Serial.print("Rear Right value: ");
    Serial.println(RRShockVal);
    Serial.print("Rear Left shock value: ");
    Serial.println(RLShockVal);

    Serial.print("Accel X: ");
    Serial.print(a.acceleration.x);
    Serial.print(" m/s^2");
    Serial.print("\tY: ");
    Serial.print(a.acceleration.y);
    Serial.print(" m/s^2 ");
    Serial.print("\tZ: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2 ");

    Serial.print("Mag X: ");
    Serial.print(m.magnetic.x);
    Serial.print(" uT");
    Serial.print("\tY: ");
    Serial.print(m.magnetic.y);
    Serial.print(" uT");
    Serial.print("\tZ: ");
    Serial.print(m.magnetic.z);
    Serial.println(" uT");

    Serial.print("Gyro X: ");
    Serial.print(g.gyro.x);
    Serial.print(" rad/s");
    Serial.print("\tY: ");
    Serial.print(g.gyro.y);
    Serial.print(" rad/s");
    Serial.print("\tZ: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");
  }

  Accelx = String(a.acceleration.x, 2);
  Accely = String(a.acceleration.y, 2);
  Accelz = String(a.acceleration.z, 2);
  Magx = String(m.magnetic.x, 2);
  Magy = String(m.magnetic.y, 2);
  Magz = String(m.magnetic.z, 2);
  Gyrox = String(g.gyro.x, 2);
  Gyroy = String(g.gyro.y, 2);
  Gyroz = String(g.gyro.z, 2);
  SteeringOut = String(SteeringVal);
  FRShockOut = String(FRShockVal);
  FLShockOut = String(FLShockVal);
  RRShockOut = String(RRShockVal);
  RLShockOut = String(RLShockVal);
  filt_ax = String(ax);
  filt_ay = String(ay);
  filt_az = String(az);

  filt_gx = String(gx);
  filt_gy = String(gy);
  filt_gz = String(gz);

  loop_count++;

  if (timer_WriteToSD.check())
  {
    samples_per_sec = loop_count/(loop_duration/1000);
    digitalWrite(LED_BLUE, HIGH);
    write_to_SD();
    loop_duration=0;
    loop_count=0;
  }

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

  if (timer_heartbeat.check())
  {
    digitalToggle(LED_BUILTIN); // turn the LED on (HIGH is the voltage level)
  }
}

void update_value_filtered(int &old_reading,int new_reading, const double ADC_ALPHA)
{
	old_reading = ADC_ALPHA * old_reading + (1-ADC_ALPHA) * new_reading;

}