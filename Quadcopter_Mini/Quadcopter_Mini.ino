#include "GLOBAL.h"
#define DEBUG_MODE false


int operation_mode = 1;


cSerial *Uart;
cNavigation *Navigation;
cImuInterface *Imu;
cController *Controller;
cCommunicationInterface *Communication;
tCommand *Command;



void setup() 
{

    Uart = new cSerial;
    Navigation = new cNavigation;
    Imu = new cMPU6050;
    Controller = new cController;
    Communication = new cESP;
    Command = new tCommand;

  ////// Controller Init
  Controller->begin();
  Controller->armMotors();
  ////// Controller Init

  ////// IMU Init
  Imu->begin();
  Imu->checkIfValid();
  ////// IMU Init

  ////// COMMUNICATION Init
  Communication->begin();
  ////// COMMUNICATION Init

  Imu->calibrate();
  Navigation->begin(Imu);
  safe_mode();
}




float time_wo_command = 0, time_wo_command_th=1;

void loop()
{

// Get Navigation Solution
Navigation->update();

//Process Command
process_command();

// Calculate the Pseudo Control 
Controller->calculatePseudoControl(Navigation,Command);

// Control Allocation
Controller->controlAllocation();


  if (DEBUG_MODE)
  {
    // Test Speed of Controller
    static int i = 0;
    static float mean_dt = 0;
    mean_dt += Navigation->dt;
    i++;
    if (i >= 1000)
    {
      mean_dt /= i;
      Serial.println(mean_dt, 5);    
      mean_dt = 0;
      i = 0;
    }
  }


}










void process_command()
{
  /////////////////////////////////////////////////////Process Command
// Check if a new commando has been received....
if ( Communication->getCommand(Command))
  {
    time_wo_command = 0;
    PORTB &= ~(1<<PB5);
    /////////////////////////////////////////////////// OFF Command received
    if (Command->T <= -100 && Command->r <= -100)
      {

                

        Controller->pseudo_control.M[0]=0;
        Controller->pseudo_control.M[1]=0;
        Controller->pseudo_control.M[2]=0;
        Controller->pseudo_control.T=0;
        // Control Allocation
        Controller->controlAllocation();

        safe_mode();
  
      }
      /////////////////////////////////////////////////// OFF Command received
  }
// ...check if too much time has passed  
else
  {
    time_wo_command += Navigation->dt;
    if (time_wo_command > time_wo_command_th)
      {
        PORTB |= (1<<PB5);
        Command->q_BI_x = 0 ;
        Command->q_BI_x = 0 ;
        Command->T = -110;
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
            Communication->getCommand(Command);
            if (Command->T <= -100 && Command->r >= 100)     // left TRIGGER + right DOWN
            start_cmd_received = true;

            if (millis()-blink_t <= 20)
            {
            PORTB |= (1<<PB5);
            }
            
            if (millis()-blink_t > 20)
            PORTB &= ~(1<<PB5);
      
            if (millis()-blink_t >= 700)
            blink_t = millis();
            
            Navigation->update();
          }
          PORTB &= ~(1<<PB5);
          blink(1);
        // Start Routine
}

