#ifndef COMM_H
#define COMM_H

#define UART_BAUD 9600

#include <Arduino.h>
#include "SoftwareSerial.h"

void blink(int n);



struct tCommand
{
    int8_t q_BI_x;
    int8_t q_BI_y;
    int8_t r;
    int8_t T;
};

class cSerial
{
public:
    void begin(long baud=UART_BAUD);
    int available();
    int getData(char *data);
    void show();
    void sendFloat(float f, char terminator = '\r');
    void sendPseudoFloat(float f, char terminator = '\r');

    int bytes;
};


 class cESP
 {
  public:
  void begin(int RX=8, int TX=9, long baud=9600);
  void createAP();
  void setupAP();
  bool isConnected();


   
  uint32_t getData(char *buffer);
  uint32_t getPayload(char *buffer);
  bool getCommand(tCommand &command);
   
  void display();
   
   
  SoftwareSerial *swSerial;
  bool detail;
   
   
 };








#endif
