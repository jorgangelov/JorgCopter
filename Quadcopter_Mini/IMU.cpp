#include "IMU.h"



void cImu::begin()
{
  // Wake Up MPU6500
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    delay(5);

    // Bypass
    Wire.beginTransmission(0x68);
    Wire.write(0x37);
    Wire.write(0x02);
    Wire.endTransmission(true);
    delay(5);

    // Mgnt Mode
    Wire.beginTransmission(0x0C);
    Wire.write(0x0A);
    Wire.write(0x02);
    Wire.endTransmission(true);
    delay(5);

    // DLPF
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

    data.bias_mx = 95;
    data.bias_my = 15;
    data.bias_mz = -15;


    time_of_calibration = 0;
    t = millis();


    
    
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


    Wire.beginTransmission(0x0C);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(0x0C,7,true);
    float mx,my,mz;
    my = (float)(Wire.read()|Wire.read()<<8);
    mx = (float)(Wire.read()|Wire.read()<<8);
    mz = -1*(float)(Wire.read()|Wire.read()<<8);
    
    if (ax*ax + ay*ay + az*az > 0.01) {
    data.ax = ax;
    data.ay = ay;
    data.az = az;
    data.temp = temp;
   
    
    data.wx = wx;
    data.wy = wy;
    data.wz = wz;

    data.mx = mx;
    data.my = my;
    data.mz = mz;
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

data.mx -= data.bias_mx;
data.my -= data.bias_my;
data.mz -= data.bias_mz;

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



cQuaternion cQuaternion::operator+(cQuaternion Q)
{
    cQuaternion Q_return;
    Q_return.values[0] = Q.values[0] + this->values[0];
    Q_return.values[1] = Q.values[1] + this->values[1];
    Q_return.values[2] = Q.values[2] + this->values[2];
    Q_return.values[3] = Q.values[3] + this->values[3];
    return Q_return;
}



cQuaternion cQuaternion::operator*(cQuaternion Q)
{
    cQuaternion Q_return;
    Q_return.values[0] = this->values[0]*Q.values[0] - this->values[1]*Q.values[1] - this->values[2]*Q.values[2] - this->values[3]*Q.values[3];
    Q_return.values[1] = this->values[1]*Q.values[0] + this->values[0]*Q.values[1] - this->values[3]*Q.values[2] + this->values[2]*Q.values[3];
    Q_return.values[2] = this->values[2]*Q.values[0] + this->values[3]*Q.values[1] + this->values[0]*Q.values[2] - this->values[1]*Q.values[3];
    Q_return.values[3] = this->values[3]*Q.values[0] - this->values[2]*Q.values[1] + this->values[1]*Q.values[2] + this->values[0]*Q.values[3];
    return Q_return;
}


cQuaternion cQuaternion::operator*(float f)
{
    cQuaternion Q_return;
    Q_return.values[0] = this->values[0] * f;
    Q_return.values[1] = this->values[1] * f;
    Q_return.values[2] = this->values[2] * f;
    Q_return.values[3] = this->values[3] * f;

    return Q_return;
}


float& cQuaternion::operator()(int i)
{
    return this->values[i-1];
}



void cQuaternion::norm()
{
    float betrag = sqrt(this->values[0]*this->values[0] + this->values[1]*this->values[1] + this->values[2]*this->values[2] + this->values[3]*this->values[3]);
    this->values[0] /= betrag;
    this->values[1] /= betrag;
    this->values[2] /= betrag;
    this->values[3] /= betrag;
}



cQuaternion cQuaternion::conjugated()
{
    cQuaternion tmp_q = *this;
    tmp_q.values[1] = -tmp_q.values[1];
    tmp_q.values[2] = -tmp_q.values[2];
    tmp_q.values[3] = -tmp_q.values[3];
    return tmp_q;
}



void cImu::update()
{
    getData();
    

    // Prediction
    cQuaternion w_bar(0,0,0,0), q_u;

    w_bar(2) = data.wx;
    w_bar(3) = data.wy;
    w_bar(4) = data.wz;


        
    
    // Update ACC
    cQuaternion w_delta(0,0,0,0);
    float ax = data.ax,ay = data.ay,az = data.az,a,m1,m2,m3, Kp = 0.1, KI=0.0025;
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



    // Update MGN
    cQuaternion w_delta_magn(0,0,0,0);
    float mx = data.mx,my = data.my,mz = data.mz,m,m1m,m2m,m3m, Kpm = 0.03;
    
    cQuaternion s_bm(0,mx,my,mz),s_iH, m_iH(0,1,0,0);
    s_iH = Q*s_bm*Q.conjugated();    
    s_iH(2) /= sqrt (s_iH(2)*s_iH(2) + s_iH(3)*s_iH(3) );
    s_iH(3) /= sqrt (s_iH(2)*s_iH(2) + s_iH(3)*s_iH(3) );     
    s_iH(4) = 0;    
    w_delta_magn = Q.conjugated()*(s_iH*m_iH)*Q;
  

     
    cQuaternion q_delta = (w_bar + w_delta*Kp+w_delta_I*KI + w_delta_magn*Kpm)*0.5*dt;
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


void cImu::magn_calibration()
{
  long start_of_calibration = millis();
  long calibration_duration = 30000;
  long currentTime = millis();

  float mx_max=-50000,mx_min=50000;
  float my_max=-50000,my_min=50000;
  float mz_max=-50000,mz_min=50000;
    Serial.println("Calibrating Magnetometer");

  while( (currentTime-start_of_calibration) < calibration_duration)
  {
    readData();

    // mx
    if (data.mx < mx_min)
    mx_min = data.mx;
    
    if (data.mx > mx_max)
    mx_max = data.mx;
    
    // my
    if (data.my < my_min)
    my_min = data.my;
    
    if (data.my > my_max)
    my_max = data.my;


    // mz
    if (data.mz < mz_min)
    mz_min = data.mz;
    
    if (data.mz > mz_max)
    mz_max = data.mz;


    
    currentTime = millis();
  }
  Serial.println(mx_max);
  Serial.println(mx_min);
  Serial.println(my_max);
  Serial.println(my_min);
  Serial.println(mz_max);
  Serial.println(mz_min);
  while(1);  
  
}


