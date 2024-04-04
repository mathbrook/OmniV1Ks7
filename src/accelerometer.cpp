#include "accelerometer.h"

bool accelerometer::init()
{
    lsm.settings.device.commInterface = IMU_MODE_I2C;
    lsm.settings.accel.scale = 16; // Set accel range to +/-16g
    lsm.settings.accel.bandwidth = 0;
    lsm.settings.accel.sampleRate = 6;

    lsm.settings.gyro.scale = 2000; // Set gyro range to +/-2000dps
    lsm.settings.gyro.sampleRate = 6;

    lsm.settings.mag.scale = 8; // Set mag range to +/-8Gs
    lsm.settings.mag.sampleRate = 6;
    // I think none of the above actually changes how the LSM gets initialized lol
    Wire.begin();
    if (!lsm.begin())
    {
        return false;
    }

    // Actually configure the lsm
    lsm.setAccelODR(6);
    lsm.setGyroODR(6);
    lsm.setMagODR(6);
    lsm.setAccelScale(16);
    lsm.setGyroScale(2000);
    lsm.setMagScale(16);
    // Run calibrations
    lsm.calibrate();
    lsm.calibrateMag();
    init_success = true;
    return true;
}

void accelerometer::run()
{
    if (init_success){
    if (lsm.accelAvailable())
    {
        lsm.readAccel();
        accelData.x = static_cast<int16_t>(lsm.calcAccel(lsm.ax) * 1000);
        accelData.y = static_cast<int16_t>(lsm.calcAccel(lsm.ay) * 1000);
        accelData.z =  static_cast<int16_t>(lsm.calcAccel(lsm.az) * 1000);
    }
    if (lsm.gyroAvailable())
    {
        lsm.readGyro();
        gyroData.x =  static_cast<int16_t>(lsm.calcGyro(lsm.gx) * 10);
        gyroData.y =  static_cast<int16_t>(lsm.calcGyro(lsm.gy) * 10);
        gyroData.z =  static_cast<int16_t>(lsm.calcGyro(lsm.gz) * 10);
    }
    if (lsm.magAvailable())
    {
        lsm.readMag();
        this->update_attitude();
    }
    }
}

// The LSM9DS1's mag x and y
// axes are opposite to the accelerometer, so my, mx are
// substituted for each other.
void accelerometer::update_attitude()
{
    float ax, ay, az, mx, my, mz;
    ax = lsm.ax;
    ay = lsm.ay;
    az = lsm.az;
    mx = -lsm.my;
    my = -lsm.mx;
    mz = lsm.mz;
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    float heading;
    if (my == 0)
        heading = (mx < 0) ? PI : 0;
    else
        heading = atan2(mx, my);

    heading -= DECLINATION * PI / 180;

    if (heading > PI)
        heading -= (2 * PI);
    else if (heading < -PI)
        heading += (2 * PI);

    // Convert everything from radians to degrees:
    heading *= 180.0 / PI;
    pitch *= 180.0 / PI;
    roll *= 180.0 / PI;
    attitudeData.x = static_cast<int16_t>(heading * 100.0);
    attitudeData.y = static_cast<int16_t>(pitch * 100.0);
    attitudeData.z = static_cast<int16_t>(roll * 100.0);
}