#include "IMU.h"



tSensor cImuInterface::data()
{
    return Sensordata_;
}

bool cImuInterface::isValid()
{
    getData();

    if (Sensordata_.ax*Sensordata_.ax + Sensordata_.ay*Sensordata_.ay + Sensordata_.az*Sensordata_.az < 1)
    {
      return false;
    }

    else
    {
      return true;
    }
}


void cImuInterface::calibrate()
{
     float gbx=0,gby=0,gbz=0;
     const uint8_t number_iter = 200;
     for (int i=0; i<number_iter; i++)
     {

       getData();

       gbx += Sensordata_.wx / ((float)number_iter);
       gby += Sensordata_.wy / ((float)number_iter);
       gbz += Sensordata_.wz / ((float)number_iter);

       delay(10);
     }

     Sensordata_.bias_wx = gbx;
     Sensordata_.bias_wy = gby;
     Sensordata_.bias_wz = gbz;

}

void cImuInterface::checkIfValid()
{
    delay(50);
    if (isValid() == false)
    {
        Serial.begin(115200);
        while (isValid() == false)
        {
          delay(1000);
          blink(3);
          Serial.println("No Imu");
        }
    }
}

void cMPU6050::begin()
{
    Wire.begin();
    Wire.setTimeout(10);

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    delay(5);
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x04); // oder 0x05
    Wire.endTransmission(true);
    delay(5);
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0);
    Wire.endTransmission(true);
}


void cMPU6050::getData()
{

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,14,true);
    float ax,ay,az,wx,wy,wz,temp;
    ax = (float)(Wire.read()<<8|Wire.read())/1800;
    ay = (float)(Wire.read()<<8|Wire.read())/1800;
    az = (float)(Wire.read()<<8|Wire.read())/1800;
    temp = (float)(Wire.read()<<8|Wire.read())/340.00+36.53;
    wx = (float)(Wire.read()<<8|Wire.read())/6900;
    wy = (float)(Wire.read()<<8|Wire.read())/6900;
    wz = (float)(Wire.read()<<8|Wire.read())/6900;

    if (ax*ax + ay*ay + az*az > 0.01) {
    Sensordata_.ax = ax;
    Sensordata_.ay = ay;
    Sensordata_.az = az;
    Sensordata_.temp = temp;


    Sensordata_.wx = wx;
    Sensordata_.wy = wy;
    Sensordata_.wz = wz;

    Sensordata_.wx -= Sensordata_.bias_wx;
    Sensordata_.wy -= Sensordata_.bias_wy;
    Sensordata_.wz -= Sensordata_.bias_wz;
    }


}






void cCustomIMU::begin()
{

}


void cCustomIMU::getData()
{

}
