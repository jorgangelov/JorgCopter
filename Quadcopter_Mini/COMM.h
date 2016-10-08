#ifndef COMM_H
#define COMM_H

#define UART_BAUD 9600

#include <Arduino.h>

void blink(int n);
void fastblink(int n);



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
    void flushBuffer();
    int bytes;
};


 class cESP
 {
  public:
  void begin(long baud=115200);
  bool isValid();
  void createAP();
  void setupAP();
  bool isConnected();
  void sendData(uint8_t *data_pt, uint8_t data_size);

  void flushBuffer();
  uint32_t getData(char *buffer);
  uint32_t getPayload(char *buffer);
  bool getCommand(tCommand &command);
   
   
   
   
   
 };








#endif
