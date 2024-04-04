#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H
#include <SparkFunLSM9DS1.h>
#include <Wire.h>

#ifndef PI
#define PI 3.1416
#endif

#define DECLINATION 5.32 // degrees, from  http://www.ngdc.noaa.gov/geomag-web/#declination

#define FLOAT_TO_INT16(value) static_cast<int16_t>(value * 100.0)

struct accelData_t
{
    int16_t x;
    int16_t y;
    int16_t z;
};
class accelerometer
{
    public:
        bool init();
        void run();
        accelData_t accelData; // Gs * 1000
        accelData_t gyroData; // Dps * 10
        accelData_t attitudeData; // Degrees * 100
    private:
        void update_attitude();
        LSM9DS1 lsm;
        bool init_success;

};

#endif
