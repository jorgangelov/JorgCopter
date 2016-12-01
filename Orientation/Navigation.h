#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "IMU.h"
#include "myMATH.h"
#define TRANSFORMATION false

void blink(int n);




class cNavigation
{
public:
    void begin(cImuInterface *_Imu);
    void update();

    cImuInterface *Imu;
    cQuaternion w_delta_I;
    cQuaternion Q;
    cQuaternion Q_IIz;
    cQuaternion Q_IzB;
    
    float dt;
    long t;
};
#endif
