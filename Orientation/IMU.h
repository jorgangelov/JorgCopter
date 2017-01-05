#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "Wire.h"
#include "COMM.h"

struct tSensor
{
  tSensor()
  {
    memset(this,0,sizeof(tSensor));
  }
    float ax;
    float ay;
    float az;

    float wx;
    float wy;
    float wz;
    float temp;

    float bias_wx;
    float bias_wy;
    float bias_wz;
};


class cImuInterface
{
public:
    virtual void begin() = 0;
    virtual void getData() = 0;

    bool isValid();
    void calibrate();
    void checkIfValid();

    tSensor data();

protected:
    tSensor Sensordata_;
};



class cMPU6050: public cImuInterface
{
public:
    void begin();
    void getData();
};


class cCustomIMU: public cImuInterface
{
public:
    void begin();
    void getData();
};



#endif
