#ifndef CTRL_H
#define CTRL_H

#include <Arduino.h>
#include "ServoTimer2.h"

#include "Navigation.h"
#include "COMM.h"

#define QUADCONFIGURATION 1 // 0 = "+"      1 = "x"
#define MOTORPIN1 5         // Front (P)    // Front-Left (P)
#define MOTORPIN2 6         // Right (N)    // Front-Right (N)
#define MOTORPIN3 10        // Rear (P)     // Rear-Right (P)
#define MOTORPIN4 11        // Left (N)     // Rear-Left  (N)

#define ESC_MIN_MS 1000
#define ESC_MAX_MS 2000


struct tPseudoControl
{
  float M[3];
  float T;
};

class cController
{
  public:
  cController()
  {
    memset(&pseudo_control,0,sizeof(tPseudoControl));
    q_e_I(1) = 0;
    q_e_I(2) = 0;
    q_e_I(3) = 0;
    q_e_I(4) = 0;
    e_r_I = 0;
    wx_F_old=0;
    wy_F_old=0;
    wz_F_old=0;
  }
  void begin();
  void armMotors();
  void calibrateMotors(long Min_ms = ESC_MIN_MS, long Max_ms = ESC_MAX_MS);
  void controlAllocation();
  void calculatePseudoControl(cNavigation *Navigation,tCommand *command);
  
  tPseudoControl pseudo_control;
  ServoTimer2 Motor1, Motor2, Motor3, Motor4;

  cQuaternion q_e_I;
  float e_r_I,wx_F_old,wy_F_old,wz_F_old;
  
};






#endif
