#include <Arduino.h>

int led = 1;

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

void setup() { 
  Serial.begin(115200);
  pinMode(led, OUTPUT);
}

void loop() {
  digitalWrite(led, HIGH);
  Serial.print("Led On  ");
  delay(500);
  digitalWrite(led, LOW);
  Serial.print("Led Off ");
  delay(500);

   Serial.print("Temperature: ");

   Serial.print((temprature_sens_read() - 32) / 1.8);
   Serial.println(" C");
   delay(5000);
}