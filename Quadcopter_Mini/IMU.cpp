#include "IMU.h"



void cImu::begin()
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

    w_delta_I(1) = 0;

    


    time_of_calibration = 0;
    t = millis();

    memset(&data,0,sizeof(data));

    
    
    uint8_t valid;
    EEPROM.get(sizeof(q_bs),valid);
    
    if (valid == 42)
    {
    EEPROM.get(0,q_bs);
    }
    
    else 
    {
      q_bs(1) = 1;
      q_bs(2) = 0;
      q_bs(3) = 0;
      q_bs(4) = 0;
    }

    
    
}


bool cImu::isValid()
{
  readData();

  if (data.ax*data.ax + data.ay*data.ay + data.az*data.az < 1)
  {
    return false;
  }

  else
  {
    return true;
  }
  
}



void cImu::readData()
{
  //////////////////////normale IMU

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
    data.ax = ax;
    data.ay = ay;
    data.az = az;
    data.temp = temp;
   
    
    data.wx = wx;
    data.wy = wy;
    data.wz = wz;
    }
    
}





void cImu::getData()
{

//////////////////////normale IMU

readData();

///////////////////////// Compensation

data.wx -= data.bias_wx;
data.wy -= data.bias_wy;
data.wz -= data.bias_wz;

////////

/////////////////////// Transformation
if (TRANSFORMATION == true)
    {
    cQuaternion q_acc_s(0,data.ax,data.ay,data.az),q_gyr_s(0,data.wx,data.wy,data.wz),q_acc_b,q_gyr_b;
    q_acc_b = q_bs*q_acc_s*q_bs.conjugated();
    q_gyr_b = q_bs*q_gyr_s*q_bs.conjugated();
    data.ax = q_acc_b(2);
    data.ay = q_acc_b(3);
    data.az = q_acc_b(4);
    data.wx = q_gyr_b(2);
    data.wy = q_gyr_b(3);
    data.wz = q_gyr_b(4);
    }
/////////////////////////////


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
}



void cImu::update()
{
    getData();
    

    // Prediction
    cQuaternion w_bar(0,0,0,0), q_u;

    w_bar(2) = data.wx;
    w_bar(3) = data.wy;
    w_bar(4) = data.wz;


        
    
    // Update
    cQuaternion w_delta(0,0,0,0);
    float ax = data.ax,ay = data.ay,az = data.az,a,m1,m2,m3, Kp = 0.2, KI=0.01;
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


void cImu::calibrate(bool cal)
{
  long current_time = millis();
  if (cal)
  {  
    if ( (current_time/1000.0)-(time_of_calibration/1000.0) > 10 )
    {
    time_of_calibration = current_time;
    blink(4);
    // Calibration
    q_bs(1) = 1;
    q_bs(2) = 0;
    q_bs(3) = 0;
    q_bs(4) = 0;
    update();
  
    const uint8_t iterations = 200;
    cQuaternion q_bs_temp(0,0,0,0);
    for (int i=0; i<iterations; i++)
    {
      update();
      q_bs_temp = q_bs_temp + Q*(1.0/iterations);
      delay(10); 
    }
    q_bs = q_bs_temp;
    EEPROM.put(0,q_bs);
    EEPROM.put(0+sizeof(q_bs),(uint8_t)42);
    
    blink(4);
    delay(500);
    }
  }
  else
  {
        if ( (current_time/1000.0)-(time_of_calibration/1000.0) > 10 )
        {
        time_of_calibration = current_time;  
        blink(4);
        q_bs(1) = 1;
        q_bs(2) = 0;
        q_bs(3) = 0;
        q_bs(4) = 0;
        EEPROM.put(0,q_bs);
        EEPROM.put(0+sizeof(q_bs),(uint8_t)42);
       
        blink(4);
        delay(500);
        }
  }
    
}


void cImu::gyro_calibration()
{
  float gbx=0,gby=0,gbz=0;
  const uint8_t number_iter = 200;
  for (int i=0; i<number_iter; i++)
  {

    readData();
    
    gbx += data.wx / ((float)number_iter);
    gby += data.wy / ((float)number_iter);
    gbz += data.wz / ((float)number_iter);

    delay(10);
  }

  data.bias_wx = gbx;
  data.bias_wy = gby;
  data.bias_wz = gbz;
  
  
}


