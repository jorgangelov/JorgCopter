
#include "GLOBAL.h"
volatile unsigned long Time_of_Last_Loop = 0;
volatile bool isActive = false;
const float dt_ms = 100;



cSerial Uart;
cImu Imu;






void setup() 
{


  
  Uart.begin(115200);
  Imu.begin();

  delay(50);
  if (Imu.isValid() == false){
  Serial.begin(115200);
  while (Imu.isValid() == false)
  {
    delay(1000);
    blink(3);
    Serial.println("No Imu");
  }}
  
    //TCCR1A = 0;        
    TCCR2B = 0;
    //TCCR2B |= (1 << CS20);
    TCCR2B |= (1 << CS21);
    //TCCR2B |= (1 << CS22);



      blink(5);
      Imu.gyro_calibration();

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

  
  Imu.update();

  Serial.write('H');  
  Uart.sendFloat(Imu.Q(1),0);
  Uart.sendFloat(Imu.Q(2),0);
  Uart.sendFloat(Imu.Q(3),0);
  Uart.sendFloat(Imu.Q(4));
  
  
    
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

