#include "Navigation.h"



void cNavigation::begin(cImuInterface *_Imu)
{
    Imu = _Imu;
    w_delta_I(1) = 0;
    t = millis();
}




void cNavigation::update()
{
    Imu->getData();
    long act = millis();
    if ((act-t) < act)
        {
        dt = (act-t)/1000.0;
        }
    else
        {
        dt = 0.001;
        }
    t = millis();

    // Prediction
    cQuaternion w_bar(0,0,0,0), q_u;

    w_bar(2) = Imu->data().wx;
    w_bar(3) = Imu->data().wy;
    w_bar(4) = Imu->data().wz;


        
    
    // Update
    cQuaternion w_delta(0,0,0,0);
    float ax = Imu->data().ax,ay = Imu->data().ay,az = Imu->data().az,a,m1,m2,m3, Kp = 0.2, KI=0.01;
    a = sqrt(ax*ax+ay*ay+az*az);
    if (a > 1 && a < 20)
    {
    m1 = ax/a;
    m2 = ay/a;
    m3 = az/a;
      
    
    cQuaternion s_b(0,-m1,-m2,-m3), g_i(0,0,0,1), g_b(0,2*Q(4)*Q(2)-2*Q(3)*Q(1),2*Q(4)*Q(3)+2*Q(2)*Q(1),1-2*Q(3)*Q(3)-2*Q(2)*Q(2));
    w_delta = s_b*g_b;
    w_delta_I = w_delta_I + w_delta*dt;
    
    }





    
    cQuaternion q_delta = (w_bar + w_delta*Kp+w_delta_I*KI)*0.5*dt;
    q_delta(1) = 1;
    Q = Q*q_delta;
    Q.norm();

    
  
    
    
    Q_IIz(1) = Q(1)/(Q(1)*Q(1)+Q(4)*Q(4));
    Q_IIz(2) = 0;
    Q_IIz(3) = 0;
    Q_IIz(4) = Q(4)/(Q(1)*Q(1)+Q(4)*Q(4));

    Q_IzB = Q_IIz.conjugated()*Q;  
    
  }




