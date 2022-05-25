# Practica 4.1. Sistemas operativos en tiempo real
## **Introducción**
El objetivo de la práctica es comprender el funcionamiento de un sistema operativo en tiempo real, para el cual generaremos varias tareas y veremos como se ejecutan dividiendo el tiempo de uso de la CPU.
## **Software y funcionamiento**
### **- Estructura del Setup**
Primero definiremos el pin al que está conectado el LED y se establecerá su modo OUTPUT. En el Setup se crea una tarea con en la que primero se tendrá su nombre, después la dimensión del stack, el parámetro, la prioridad, y el seguimiento de la tarea.
```cpp
#include <Arduino.h>
void anotherTask( void * parameter );
void setup()
{
  Serial.begin(112500);
  /* we create a new task here */
  xTaskCreate(anotherTask,"another Task",10000,NULL,1,NULL); 
}
```
### **- Estructura del Loop**
En el Loop se crea una función loop que va repitiendo *"this is ESP32 Task"*, posterior a un delay de 1000 ms, se genera otro void llamado *"AnotherTask"* que lo que hará es un println diciendo *"This is another Task"* también con un delay de 1000 ms.
```cpp
void loop()
{
  Serial.println("this is ESP32 Task");
  delay(1000);
}
/* this function will be invoked when additionalTask was created */
void anotherTask( void * parameter )
{
  /* loop forever */
  for(;;)
  {
  Serial.println("this is another Task");
  delay(1000);
  }
}
```
#### **- Funcionamiento**
![](Practica04_1.png)
## **código completo**
```cpp
#include <Arduino.h>

void anotherTask( void * parameter );

void setup()
{
  Serial.begin(112500);
  /* we create a new task here */
  xTaskCreate(anotherTask,"another Task",10000,NULL,1,NULL); 
}

void loop()
{
  Serial.println("this is ESP32 Task");
  delay(1000);
}
/* this function will be invoked when additionalTask was created */
void anotherTask( void * parameter )
{
  /* loop forever */
  for(;;)
  {
  Serial.println("this is another Task");
  delay(1000);
  }
}
```