#include "CTRL.h"

void cController::begin()
{

Motor1.attach(MOTORPIN1);
Motor2.attach(MOTORPIN2);
Motor3.attach(MOTORPIN3);
Motor4.attach(MOTORPIN4);
}

void cController::armMotors()
{
Motor1.write(ESC_MIN_MS);
Motor2.write(ESC_MIN_MS);
Motor3.write(ESC_MIN_MS);
Motor4.write(ESC_MIN_MS);
}

void cController::calibrateMotors(long minms, long maxms)
{
cSerial Uart;
Uart.begin(115200);
Serial.println("Press anything to send max and then plug in Motors");
Uart.flushBuffer();
while ( ! Uart.available());
fastblink(4);
Motor1.write(maxms);
Motor2.write(maxms);
Motor3.write(maxms);
Motor4.write(maxms);
Serial.println("Press anything to send min");
Uart.flushBuffer();
while ( ! Uart.available());
Motor1.write(minms);
Motor2.write(minms);
Motor3.write(minms);
Motor4.write(minms);
fastblink(4);
Serial.println("Done");
}


void cController::controlAllocation()
{
  
    float n1=0,n2=0,n3=0,n4=0,l=pseudo_control.M[0],m=pseudo_control.M[1],n=pseudo_control.M[2],k_M=100,k_T=40,k_D=120;
    // 
    if (QUADCONFIGURATION == 0)
    {
    n1 = k_T*pseudo_control.T + m*k_M - n*k_D;
    n2 = k_T*pseudo_control.T - l*k_M + n*k_D;
    n3 = k_T*pseudo_control.T - m*k_M - n*k_D;
    n4 = k_T*pseudo_control.T + l*k_M + n*k_D;
    }

    else if (QUADCONFIGURATION == 1)
    {
    // x
    n1 = k_T*pseudo_control.T + 0.707*m*k_M + 0.707*l*k_M - n*k_D;
    n2 = k_T*pseudo_control.T + 0.707*m*k_M - 0.707*l*k_M + n*k_D;
    n3 = k_T*pseudo_control.T - 0.707*m*k_M - 0.707*l*k_M - n*k_D;
    n4 = k_T*pseudo_control.T - 0.707*m*k_M + 0.707*l*k_M + n*k_D;
    }
    
    if (n1 < 0) n1 = 0;
    if (n2 < 0) n2 = 0;
    if (n3 < 0) n3 = 0;
    if (n4 < 0) n4 = 0;

    if (n1 > 1000) n1 = 1000;
    if (n2 > 1000) n2 = 1000;
    if (n3 > 1000) n3 = 1000;
    if (n4 > 1000) n4 = 1000;

long servo1,servo2,servo3,servo4;
servo1 = n1 + 1000;
servo2 = n2 + 1000;
servo3 = n3 + 1000;
servo4 = n4 + 1000;



Motor1.write(servo1);
Motor2.write(servo2);
Motor3.write(servo3);
Motor4.write(servo4);


    
}




void cController::calculatePseudoControl(cImu* Imu,tCommand* command)
{
cQuaternion q_e(1,0,0,0);

// Gains
float I_enable = 1;
float Kp=175, Kd=40, KI=40, Kdd = 2.5;
// Gains
float wdot[3], T;
const int8_t I_TH = 0;

// q_BIz_d
q_BIz_d(2) = (command->q_BI_x)/300.0;
q_BIz_d(3) = (command->q_BI_y)/300.0;
q_BIz_d(1) = sqrt(1 - q_BIz_d(2)*q_BIz_d(2) - q_BIz_d(3)*q_BIz_d(3));


////// WDOT
float wdot_freq = 15, wx_F=0, wy_F=0, wz_F=0, wxdot_F=0, wydot_F=0, wzdot_F=0;
float K_wdot = 1 + wdot_freq*Imu->dt;
wx_F = (1/K_wdot)*wx_F_old + (wdot_freq*Imu->dt/K_wdot)*Imu->data.wx;
wxdot_F = (wx_F-wx_F_old)/Imu->dt;
wx_F_old = wx_F;

wy_F = (1/K_wdot)*wy_F_old + (wdot_freq*Imu->dt/K_wdot)*Imu->data.wy;
wydot_F = (wy_F-wy_F_old)/Imu->dt;
wy_F_old = wy_F;

////// WDOT



//q_e
q_e = (q_BIz_d*q_IzI_d)*Imu->Q;

//q_e_I
if (I_enable > 0)
q_e_I = q_e_I + (q_e*Imu->dt*(command->T > I_TH));

if (command->T <= -100)
{
q_e_I = q_e_I*0;
}

wdot[0] = -2*(Kp*q_e(2) + KI*q_e_I(2) + Kd*0.5*Imu->data.wx ) -Kdd*wxdot_F;
wdot[1] = -2*(Kp*q_e(3) + KI*q_e_I(3) + Kd*0.5*Imu->data.wy ) -Kdd*wydot_F;
wdot[2] = -2*(Kp*q_e(4) + KI*q_e_I(4) + Kd*0.5*Imu->data.wz );

pseudo_control.M[0] = (wdot[0]/100) * (command->T >= -100);
pseudo_control.M[1] = (wdot[1]/100) * (command->T >= -100);
pseudo_control.M[2] = (wdot[2]/80) * (command->T >= -100);
pseudo_control.T = (command->T+110)/10.0;




}


