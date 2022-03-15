#include <Arduino.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

const int led_1 = 32;
const int led_2 = 25;
const double Potter = 34;

void Task1code( void * parameter ){
  Serial.print("Task1 is running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    digitalWrite(led_1, HIGH);
    vTaskDelay(500);
    digitalWrite(led_1, LOW);
    vTaskDelay(500);
  } 
}

void Task2code( void * parameter ){
  Serial.print("Task2 is running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    digitalWrite(led_2, HIGH);
    vTaskDelay(1000);
    digitalWrite(led_2, LOW);
    vTaskDelay(1000);
  }
}

void Task3code(void * parameter)
{
  Serial.print("Task3 is running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    double prova = analogRead(Potter);
  Serial.print("El valor del voltaje es ");
  Serial.println(prova);
  vTaskDelay(1000);
  }

}

void setup() {
  Serial.begin(115200); 
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  

  xTaskCreatePinnedToCore(Task1code,"Task1",10000,NULL,1,&Task1,0);                         
  vTaskDelay(500); 

  xTaskCreatePinnedToCore(Task2code,"Task2",10000,NULL,1,&Task2,1);          
    vTaskDelay(500); 

  xTaskCreatePinnedToCore(Task3code,"Task3",10000,NULL,1,&Task3,1);          
    vTaskDelay(5000); 
}

void loop() {
  
}