#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "EEPROM.h"
#include "Wire.h"
#include "myMATH.h"

#define TRANSFORMATION false

void blink(int n);



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

    float mx;
    float my;
    float mz;
    
    float temp;

    float bias_wx;
    float bias_wy;
    float bias_wz;

    float bias_mx;
    float bias_my;
    float bias_mz;
};


class cImu
{
public:
    void begin();
    void update();
    bool isValid();
    void calibrate(bool cal=true);
    void gyro_calibration();
    void magn_calibration();
    tSensor data;
    cQuaternion w_delta_I;

    cQuaternion Q;
    cQuaternion Q_IIz;
    cQuaternion Q_IzB;
    
    float dt;
    cQuaternion q_bs;
    void readData();
    void getData();
    long t;
    long time_of_calibration;
};
#endif
