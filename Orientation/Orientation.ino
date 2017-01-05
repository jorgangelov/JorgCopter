#include "COMM.h"
#include "Navigation.h"
volatile unsigned long Time_of_Last_Loop = 0;
volatile bool isActive = false;
const float dt_ms = 100;



cSerial *Uart;
cImuInterface *Imu;
cNavigation *Navigation;





void setup() 
{
    Uart = new cSerial;
    Imu = new cMPU6050;
    Navigation = new cNavigation;

  
  Uart->begin(115200);
  Imu->begin();
  Imu->checkIfValid();

  
    //TCCR1A = 0;        
    TCCR2B = 0;
    //TCCR2B |= (1 << CS20);
    TCCR2B |= (1 << CS21);
    //TCCR2B |= (1 << CS22);



      blink(5);
      Imu->calibrate();
      Navigation->begin(Imu);


    TIMSK2 = (1 << TOIE2);


}



int i = 0;


void loop()
{


}


ISR(TIMER2_OVF_vect)
{
  if ( (millis()-Time_of_Last_Loop) >= dt_ms && !isActive)
  {
  isActive = true;
  Time_of_Last_Loop = millis();
  sei();  

  
  Navigation->update();

  Serial.write('H');  
  Uart->sendFloat(Navigation->Q(1),0);
  Uart->sendFloat(Navigation->Q(2),0);
  Uart->sendFloat(Navigation->Q(3),0);
  Uart->sendFloat(Navigation->Q(4));

/*
  Serial.write('H');  
  Uart.sendFloat(Imu.data.ax,0);
  Uart.sendFloat(Imu.data.ay,0);
  Uart.sendFloat(Imu.data.az,0);
  Uart.sendFloat(Imu.data.wy);


Serial.print(Imu.data.mx);
Serial.print(" ");
Serial.print(Imu.data.my);
Serial.print(" ");
Serial.print(Imu.data.mz);
Serial.print(" ");
Serial.print(sqrt(Imu.data.mx*Imu.data.mx+Imu.data.my*Imu.data.my+Imu.data.mz*Imu.data.mz));
Serial.println("");

*/  

  
  
  isActive=false;
  
  }

}

