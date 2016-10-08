#include "GLOBAL.h"


int operation_mode = 1;


cSerial Uart;
cImu Imu;
cController Controller;
cESP Esp;

tCommand command = {0,0,0,0};



void setup() 
{
pinMode(A0,INPUT);
digitalWrite(A0,1);
operation_mode = digitalRead(A0);


if (operation_mode == 1)
  {
  ////// Controller Init
  Controller.begin();  
  Controller.armMotors();  
  ////// Controller Init

  ////// IMU Init
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
  ////// IMU Init


  ////// ESP Init
  Esp.begin();
  delay(50);
  if (Esp.isValid() == false){
   Serial.begin(115200);
  while (Esp.isValid() == false)
  {
    delay(1000);
    blink(3);
    Serial.println("No Esp");
  }}

  
  Esp.setupAP();
  blink(2);
  while( !Esp.isConnected() )
  {
    delay(100);
  }
  blink(3);
  ////// ESP Init

  
  Imu.gyro_calibration();
  safe_mode();

  
  }
  
  
//////////////////////////////////////////////// DEBUG MODE
if (operation_mode == 0)
  {
  Serial.begin(115200);
  Serial.println("Setup Mode...");  
  blink(3);
  while(operation_mode == 0)
      {
        

    
      }
  
  }  
//////////////////////////////////////////////// DEBUG MODE


}




float time_wo_command = 0, time_wo_command_th=1;

void loop()
{
// Get Sensor Data and Attitude  
Imu.update();  

//Process Command
process_command();

// Calculate the Pseudo Control 
Controller.calculatePseudoControl(&Imu,&command);

// Control Allocation
Controller.controlAllocation();

/*
// Test Speed of Controller
static int i = 0;
static float mean_dt = 0;
mean_dt += Imu.dt;
i++;
if (i >= 100)
{
  mean_dt /= i;
  Serial.println(mean_dt,5);
  mean_dt = 0;
  i = 0;
}
*/


}










void process_command()
{
  /////////////////////////////////////////////////////Process Command
// Check if a new commando has been received....
if ( Esp.getCommand(command))
  {
    time_wo_command = 0;
    PORTB &= ~(1<<PB5);
    /////////////////////////////////////////////////// OFF Command received
    if (command.T <= -100 && command.r <= -100)
      {

                

        Controller.pseudo_control.M[0]=0;
        Controller.pseudo_control.M[1]=0;
        Controller.pseudo_control.M[2]=0;
        Controller.pseudo_control.T=0;
        // Control Allocation
        Controller.controlAllocation();

        safe_mode();
  
      }
      /////////////////////////////////////////////////// OFF Command received
  }
// ...check if too much time has passed  
else
  {
    time_wo_command += Imu.dt;
    if (time_wo_command > time_wo_command_th)
      {
        PORTB |= (1<<PB5);
        command.q_BI_x = 0 ;
        command.q_BI_x = 0 ;
        command.T = -110;
      }
    
  }
/////////////////////////////////////////////////////Process Command
  
}


void safe_mode()
{
          // Start Routine
        bool start_cmd_received = false;
        long blink_t = millis();
        DDRB |= (1<<PB5);

        while(!start_cmd_received)
          {
            Esp.getCommand(command);
            if (command.T <= -100 && command.r >= 100)     // left TRIGGER + right DOWN
            start_cmd_received = true;

            if (command.q_BI_y <= -100 && command.r >= 100) // left DOWN + right DOWN 
            Imu.calibrate();

            if (command.q_BI_y >= 100 && command.r <= -100) // left UP + right UP
            Imu.calibrate(false);
            
            
            if (millis()-blink_t <= 20)
            {
            PORTB |= (1<<PB5);
            }
            
            if (millis()-blink_t > 20)
            PORTB &= ~(1<<PB5);
      
            if (millis()-blink_t >= 700)
            blink_t = millis();
            
            Imu.update();
          }
          PORTB &= ~(1<<PB5);
          blink(1);
        // Start Routine
}

