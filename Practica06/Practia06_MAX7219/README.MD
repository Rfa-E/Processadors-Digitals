# Practica 6.2. Dot LED Matrix Display
## **Introducción**
Como en la primera parte de la práctica utilizaremos periféricos con capacitados para usarlos con el bus SPI (Serial Peripheral Interface). En este caso emplearemos una matriz de leds que en conjunto se pueden hacer servir como una pantalla (o display). 
## **Hardware**
* ESP32-Wroom-32.
* MAX7219 Matrix Display.
* Protoboard (opcional).
## **Software y su funcionamiento**
### **- Cabecera del código**
Como es habitual en los últimos códigos, iniciamos incluyendo las librerías necesarias para el código, en este caso, principalmente incluiremos la librería que nos permitirá utilizar las funciones SPI, "SPI.h", posteriormente incluimos las librerías que necesitara el periférico, en este caso, "MD_Parola.h" y "MD_MAX72xx.h".

A continuación, definimos una variable según el tipo de nuestro hardware, una otra variable para establecer el número de pantallas que tenemos disponibles y por último su pin de uso.
```cpp
#include <Arduino.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// Uncomment according to your hardware type
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
//#define HARDWARE_TYPE MD_MAX72XX::GENERIC_HW

// Defining size, and output pins
#define MAX_DEVICES 4
#define CS_PIN 5

MD_Parola Display = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
```
### **- Estructura del Setup**
En el setup declararemos tres funciones, la primera, es para iniciar el display, el cual no necesita de parámetros. La segunda, definiremos la intensidad de los leds, cambiando esta también podemos cambiar el color de los leds. Y por último, la tercera, será para limpiar el bus del display de cualquier información.
```cpp
void setup()
{
 
  Display.begin();
  Display.setIntensity(0);
  Display.displayClear();
}
```
### **- Estructura del Loop**
En la estructura del Loop declararemos la alineación del texto con la función "setTextAlignment". También, usaremos la función "print" para enviar el mensaje que queramos mostrar por pantalla. Para que los mensajes no se solapen le colocamos un delay de 2sec para también visualizarlos con mayor claridad. En el último mensaje haremos uso de la función "setInvert(true)" que será para invertir los colores de mensaje.
```cpp
void loop()
{
  Display.setTextAlignment(PA_CENTER);
  Display.print("UPC");
  delay(2000);
  
  Display.setTextAlignment(PA_CENTER);
  Display.print("eseiaat");
  delay(2000);

  Display.setTextAlignment(PA_CENTER);
  Display.print("Rafael");
  delay(2000);

  Display.setTextAlignment(PA_CENTER);
  Display.setInvert(true);
  Display.print("CopyRight");
  delay(2000);

  Display.setInvert(false);
  delay(2000);
}
```
### **- Funcionamiento**
Se puede visualizar el funcionamiento del circuito con el vídeo adjuntado.
## **Código completo**
```cpp
#include <Arduino.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// Uncomment according to your hardware type
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
//#define HARDWARE_TYPE MD_MAX72XX::GENERIC_HW

// Defining size, and output pins
#define MAX_DEVICES 4
#define CS_PIN 5

MD_Parola Display = MD_Parola(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);

void setup() 
{ 
  Display.begin();
  Display.setIntensity(0);
  Display.displayClear();
}

void loop() 
{
  Display.setTextAlignment(PA_CENTER);
  Display.print("UPC");
  delay(2000);
  
  Display.setTextAlignment(PA_CENTER);
  Display.print("eseiaat");
  delay(2000);

  Display.setTextAlignment(PA_CENTER);
  Display.print("Rafael");
  delay(2000);

  Display.setTextAlignment(PA_CENTER);
  Display.setInvert(true);
  Display.print("CopyRight");
  delay(2000);

  Display.setInvert(false);
  delay(2000);
}
```