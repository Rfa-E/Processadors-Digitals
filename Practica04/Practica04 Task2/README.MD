# Practica 4.2.
## **Introducción**
Realizaremos un programa que utilice dos tareas, una enciende un led y otra lo apaga, dichas tareas deben estar sincronizadas.

Posteriormente, también añadiremos un potenciómetro para visualizar cuál era el nivel de tensión que hay en el circuito.
## **Software y su funcionamiento**
### **- Declaracion de constantes**
Al inicio del código utilizaremos las funciones *"TaskHandle_t"* para declarar 3 tareas, que son las posteriores a encender/apagar los leds, y leer los valores del potenciómetro.

Luego declaramos las constantes para fijar los pines de los leds y el potenciómetro.
```cpp
#include <Arduino.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

const int led_1 = 32;
const int led_2 = 25;
const double Potter = 34;
```
### **- Void Task1code y Task2code**
En ambas funciones se inician escribiendo por pantalla un *"Task1/2 is runnig on core "* y posteriormente con la función *"xPortGetCoreID()"* indicaremos cuál de los dos cores está asignado la función. A continuación, utilizaremos un *"digitalWrite(led_1/2, HIGH/LOW)"* para encender u apagar los leds.
```cpp
void Task1code( void * parameter )
{
  Serial.print("Task1 is running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    digitalWrite(led_1, HIGH);
    vTaskDelay(500);
    digitalWrite(led_1, LOW);
    vTaskDelay(500);
  } 
}

void Task2code( void * parameter )
{
  Serial.print("Task2 is running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    digitalWrite(led_2, HIGH);
    vTaskDelay(1000);
    digitalWrite(led_2, LOW);
    vTaskDelay(1000);
  }
}
```
### **- Void Task3code**
Al igual que en las funciones anteriores, comenzamos escribiendo por pantalla *"Task3 is runnig on core "* y con el *"xPortGetCoreID()"* indicaremos cuál es el core en el que trabaja. A continuación, definiremos una variable double prova, para leer el valor analógico que nos proporciona el potenciómetro, el cual después imprimiremos por pantalla.
```cpp
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
```
### **- Estructura del Setup**
Definiremos los pines a los que están conectados ambos leds y los estableceremos en modo OUTPUT. Posteriormente, crearemos las tareas correspondientes en las que primero ira el nombre, la dimensión del stack, el parámetro, la prioridad, y el seguimiento de la tarea.
```cpp
void setup()
{
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
```
### **- Funcionamiento**

**Ver los vídeos incluidos del montaje.**

## **código completo**
```cpp
#include <Arduino.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

const int led_1 = 32;
const int led_2 = 25;
const double Potter = 34;

void Task1code( void * parameter )
{
  Serial.print("Task1 is running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    digitalWrite(led_1, HIGH);
    vTaskDelay(500);
    digitalWrite(led_1, LOW);
    vTaskDelay(500);
  } 
}

void Task2code( void * parameter )
{
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

void setup()
{
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

void loop(){ }
```