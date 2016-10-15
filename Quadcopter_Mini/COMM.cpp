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
    bytes = 0;
    bool TimedOut = false;
    unsigned long startTime;
    while ( !TimedOut )
    {
      if (available())
      {
        data[bytes] = Serial.read();
        bytes++;
        startTime = millis();
      }
      if ( ( millis() - startTime) > TIMEOUT_MS)
      {
        TimedOut = true;
      }
    }
    return bytes;
  }
  return 0;
}


void cSerial::show()
{
  if (this->available())
  {
    char* data = new char [this->available()];
    bytes = this->available();
    this->getData(data);
    data[bytes] = '\0';
    Serial.println(data);
    delete data;
  }
}


void cSerial::sendFloat(float f, char terminator)
{
  char *bytept = (char*)&f;
  Serial.write(*bytept);
  Serial.write(*(bytept + 1));
  Serial.write(*(bytept + 2));
  Serial.write(*(bytept + 3));
  if (terminator != 0)
    Serial.print(terminator);
}


void cSerial::sendPseudoFloat(float f, char terminator)
{
  Serial.write((int8_t)(f * 128.0));

  if (terminator != 0)
    Serial.print(terminator);
}


void cSerial::flushBuffer()
{
  while (Serial.available())
  {
    Serial.read();
  }
}


void cESP::begin(long baud)
{


  Serial.begin(baud);
  flushBuffer();
  Serial.println("AT+RST");
  delay(5000);
  //Serial.println("AT+CIOBAUD=115200");

  /*
       swSerial->println("AT+CIOBAUD=9600");
       if (detail)
       display();
       swSerial->begin(9600);
       if (detail)
       display();

  */




}




void cESP::flushBuffer()
{
  while (Serial.available())
  {
    Serial.read();
  }
}

uint32_t cESP::getData(char *mybuffer)
{
  int i = 0;
  if (Serial.available())
  {
    while (Serial.available())
    {
      mybuffer[i] = Serial.read();
      i = i + 1;
    }
  }

  return i;

}







void cESP::createAP()
{

  Serial.println("AT+CWMODE=3");
  delay(1000);

  Serial.println("AT+CWSAP=\"JORG_QUADCOPTER\",\"bebosbebos\",5,3");
  delay(1000);
  flushBuffer();

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
  Serial.println("AT+CIPMUX=1");
  delay(400);
  Serial.println("AT+CIPSERVER=1,1337");
  delay(400);
  Serial.println("AT+CIPSTART=1,\"UDP\",\"192.168.4.100\",1338");
  flushBuffer();
  /////////////////

}

void cESP::sendData(uint8_t *data_pt, uint8_t data_size)
{
  char line[50];
  sprintf(line, "AT+CIPSEND=1,%u", data_size);
  Serial.println(line);
  Serial.write(data_pt, data_size);
}



bool cESP::isConnected()
{
  flushBuffer();
  Serial.println("AT+CWLIF");
  delay(50);
  char mybuffer[100];
  uint32_t len = getData(mybuffer);
  if (len > 30)
  {
    return true;
  }
  else
  {
    return false;
  }


}


bool cESP::isValid()
{
  char mybuffer[100];
  uint32_t len = getData(mybuffer);
  if (len > 30)
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


  while (Serial.available() > 6 && start_received != true)
  {
    if (Serial.read() == ':')
    {
      start_received = true;
    }

  }



  if (start_received == true)
  {
    int i;
    for (i = 0; i < 6; i++)
    {
      buffer[i] = Serial.read();
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
  char payload[6];
  int len = getPayload(payload);
  if (len == 6)
  {
    if (payload[0] == 126 && payload[5] == 127)
    {
      command.q_BI_x = payload[1];
      command.q_BI_y = payload[2];
      command.r = payload[3];
      command.T = payload[4];
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
  DDRB |= (1 << PB5);


  for (int i = 0; i < n; i++)
  {
    PORTB |= (1 << PB5);
    delay(200);
    PORTB &= ~(1 << PB5);
    delay(200);
  }
}


void fastblink(int n)
{
  DDRB |= (1 << PB5);


  for (int i = 0; i < n; i++)
  {
    PORTB |= (1 << PB5);
    delay(100);
    PORTB &= ~(1 << PB5);
    delay(100);
  }
}
