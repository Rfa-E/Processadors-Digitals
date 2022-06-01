# Practica 7.1. Reproduccion desde memoria interna
## **Introducción**
Los datos de sonido se almacenan como una matriz en la RAM interna del ESP32. Usamos la placa de conexión de audio MAX98357 I2S para decodificar la señal digital en una señal analógica. Por lo tanto, utilizamos el protocolo I2S para generar los datos de sonido digital sin perdidas de calidad.
## **Hardware**
* ESP32-Wroom-32.
* El DAC MAX98357 (I2S).
* Un altavoz pasivo de 3 watts.
## **Software y su funcionamiento**
### **- Cabecera del codigo**
Como al comendamiento de las últimas prácticas, añadimos las cabeceras correspondientes que son básicamente librerías de audio. También se definen 3 punteros, uno que servirá para leer el archivo de audio, el segundo servirá para descodificar el archivo y el último para enviar de forma analógica el audio.
```cpp
#include "Arduino.h"
#include "FS.h"
#include "SPIFFS.h"
#include "SD.h"
#include "AudioGeneratorAAC.h"
#include "AudioOutputI2S.h"
#include "AudioFileSourcePROGMEM.h"
#include "sampleaac.h"

AudioFileSourcePROGMEM *in;
AudioGeneratorAAC *aac;
AudioOutputI2S *out;
```
### **- Estructura del Setup**
En la estructura del setup se inicializa un puntero "in" con los parámetros de la muestra del archivo del audio aac y el tamaño del archivo. Definiremos otro puntero que será el encargado de codificar en el aac y el out del audio de salida. Se le inicializa con una ganancia de 0.125 y se definen también los pines.
```cpp
void setup()
{
  Serial.begin(115200);

  in = new AudioFileSourcePROGMEM(sampleaac, sizeofsampleaac);
  aac = new AudioGeneratorAAC();
  out = new AudioOutputI2S();
  out -> SetGain(0.125);
  out -> SetPinout(26,25,22);
  aac->begin(in, out);
}
```
### **- Estructura del Loop**
En la estructura del loop se comprueba que el aac se ejecute hasta estar completamente decodificada. Cuando termine el proceso se finalizara y se imprimirá por pantalla el siguiente mensaje "Sound Generator".
```cpp
void loop()
{
  if (aac->isRunning()) 
  {
    aac->loop();
  } 
  else 
  {
    aac -> stop();
    Serial.printf("Sound Generator\n");
    delay(1000);
  }
}
```
### **- Funcionamiento**

## **Codigo completo**
```cpp
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SD.h>
#include <AudioGeneratorAAC.h>
#include <AudioOutputI2S.h>
#include <AudioFileSourcePROGMEM.h>
#include <sampleaac.h>

AudioFileSourcePROGMEM *in;
AudioGeneratorAAC *aac;
AudioOutputI2S *out;

void setup()
{
  Serial.begin(115200);

  in = new AudioFileSourcePROGMEM(sampleaac, sizeofsampleaac);
  aac = new AudioGeneratorAAC();
  out = new AudioOutputI2S();
  out -> SetGain(0.125);
  out -> SetPinout(26,25,22);
  aac->begin(in, out);
}

void loop()
{
  if (aac->isRunning()) 
  {
    aac->loop();
  } 
  else 
  {
    aac -> stop();
    Serial.printf("Sound Generator\n");
    delay(1000);
  }
}
```