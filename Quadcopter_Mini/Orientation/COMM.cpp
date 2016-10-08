#include "COMM.h"


void cSerial::begin(long baud)
{
    Serial.begin(baud);
}


int cSerial::available()
{
    return Serial.available();
}


int cSerial::getData(char *data)
{
    if (this->available())
            {
            delay(50);
            bytes = 0;
            while (this->available())
                {
                data[bytes] = Serial.read();
                bytes++;
                }
            return bytes;
            }
return 0;
}


void cSerial::show()
{
    if (this->available())
        {
        char *data = new char [this->available()];
        bytes = this->available();
        this->getData(data);
        data[bytes] = '\0';
        Serial.println(data);
        }
}


void cSerial::sendFloat(float f, char terminator)
{
    char *bytept = (char*)&f;
    Serial.write(*bytept);
    Serial.write(*(bytept+1));
    Serial.write(*(bytept+2));
    Serial.write(*(bytept+3));
    if (terminator != 0)
    Serial.print(terminator);
}


void cSerial::sendPseudoFloat(float f, char terminator)
{
    Serial.write((int8_t)(f*128.0));

    if (terminator != 0)
    Serial.print(terminator);
}




void cESP::begin(int RX, int TX, long baud)
{

    detail = false;
    
    if (detail)
    Serial.begin(9600);

    swSerial = new SoftwareSerial(RX,TX);
    swSerial->begin(baud);
    swSerial->println("AT+RST");

    
    if (detail)
    display();

    delay(3000);
/*
     swSerial->println("AT+CIOBAUD=9600");
     if (detail)
     display();
     swSerial->begin(9600);
     if (detail)
     display();

*/
    
    swSerial->flush();



}



uint32_t cESP::getData(char *buffer)
{
  int i =0;
   if (swSerial->available())
  {     
      while(swSerial->available())
      {
        buffer[i] = swSerial->read();
        i = i+1;
      }
  }
  
  return i;  
  
}



void cESP::display()
{
char buffer[1000];  
delay(100);

uint32_t len = this->getData(buffer);
if (len > 0)
{
buffer[len] = '\0';
Serial.println(buffer);
}


}



void cESP::createAP()
{
  
swSerial->println("AT+CWMODE=3");
delay(500);  
if (detail)
display();
swSerial->println("AT+CWSAP=\"JORG_QUAD\",\"bebosbebos\",5,3");
delay(500);
if (detail)
display();

}

void cESP::setupAP()
{


//black
//////////////////////
/*
swSerial->println("AT+CIPMUX=0");
if (detail)
display();
delay(300);
swSerial->println("AT+CIPSTART=\"UDP\",\"0\",0,1337,2");
if (detail)
display();
delay(300);
swSerial->flush();
*/
/////////////////









//blue
//////////////////////
swSerial->println("AT+CIPMUX=1");
if (detail)
display();
delay(1000);
swSerial->println("AT+CIPSERVER=1,1337");
if (detail)
display();
delay(1000);
swSerial->flush();
/////////////////

}


bool cESP::isConnected()
{
swSerial->flush();  
swSerial->println("AT+CWLIF");
delay(50);
char buffer[100];  
uint32_t len = this->getData(buffer);
if (len > 20)
  {
    return true;
  }
else
  {
    return false;
  }


}



uint32_t cESP::getPayload(char *buffer)
{
  bool start_received = false;

  
  while (swSerial->available() > 6 && start_received != true)
  {
    if (swSerial->read() == ':')
    {
      start_received = true;
    }
    
  }


  
  if (start_received == true)
  {
    int i;
    for (i=0; i<6; i++)
    {
    buffer[i] = swSerial->read();
    }
    return i;
  }

  else
  {
    return 0;
  }
  


    
  
}



bool cESP::getCommand(tCommand &command)
{
  char payload[50];
  int len = this->getPayload(payload);
  if (len == 6)
  {
    if(payload[0] == 126 && payload[5] == 127)
    {
      command.q_BI_x = payload[1];
      command.q_BI_y = payload[2];
      command.r = payload[3];
      command.T = payload[4];
      swSerial->flush(); 
      return true;
    }
    else
    {
      return false; 
    }
  }
  else
  {
    return false;
  }
  
}







void blink(int n)
{
  DDRB |= (1<<PB5);


for (int i=0;i<n;i++)
  {
  PORTB |= (1<<PB5);
  delay(200);
  PORTB &= ~(1<<PB5);
  delay(200);
  }
}
