#include <Arduino.h>

#define RXD0 34
#define TXD0 35
#define TXD2 28

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD0, TXD2);
  Serial2.begin(115200, SERIAL_8N1, TXD2, TXD0);
}

void loop()
{ 
  if(Serial.available())
    {
     Serial1.write(Serial.read());  
    }
   if(Serial1.available())
    {
     Serial2.write(Serial1.read());
     Serial.printf("Datos");  
    }
}