 # Practica 7.2. Reproducir un archivo desde una tarjeta SD
## **Introducción**
En esta práctica lo que queremos es reproducir un archivo Wave o mp3 a través del ESP32 y el altavoz. Debido a que el ESP32 tiene que leer el archivo y reenviar la señal de audio digital al MAX98357A, tenemos que usar una tarjeta SD.
## **Hardware**
* ESP32-Wroom-32.
* El DAC MAX98357 (I2S).
* El lector de tarjeta SD.
* Un altavoz pasivo de 3 watts.
## **Software y su funcionamiento**
### **- Cabecera del código**
Como en la práctica anterior, añadimos la librería correspondiente para hacer uso de la Tarjeta SD "SD.h" y la librería "FS.h" que es básicamente una librería de control de audio. A continuación, se definen los pones que se utilizaran para conectar los componentes. Los primeros 4 pines son para conectar el lector de SD y los siguientes para conectar el DAC MAX9835. También definimos un objeto Audio.
```cpp
#include "Audio.h"
#include "SD.h"
#include "FS.h"

// Digital I/O used
// Tarjeta SD
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
// DAC MAX9835
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26

Audio audio;
```
### **- Estructura del Setup**
En la estructura del Setup, se inicializa el pin "SD_CS" como salida, y a nivel alto, después iniciamos el bus SPI con sus tres pines "MOSI, MISO, SCK". Iniciamos la velocidad de comunicación en, 115200 ms, e iniciamos la Tarjeta SD con su pin "SD_CS". Posteriormente, en el objeto audio, definimos los pines de salida "BCLK, LRC, DOUT", se sitúa el volumen en un valor de 10 (se puede modificar entre los valores de {0...21}), y realiza la conexión para escoger el archivo de la tarjeta SD.
```cpp
void setup()
{
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.begin(115200);
    SD.begin(SD_CS);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(10); // 0...21
    audio.connecttoFS(SD, "audio.wav");
}
```
### **- Estructura del Loop**
Finalmente, en el loop simplemente se llama la función de audio en un loop. Las funciones que aparecen abajo se podrían utilizar de forma opcional para mostrar por pantalla información que podría ser relevante como el nombre, el tiempo, cuando llega al final, etc.
```cpp
void loop()
{
    audio.loop();
}

// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreaminfo(const char *info){
    Serial.print("streaminfo  ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print("eof_speech  ");Serial.println(info);
}
```
## **Código completo**
```cpp
#include "Audio.h"
#include "SD.h"
#include "FS.h"

// Digital I/O used
// Tarjeta SD
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
// DAC MAX9835
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26

Audio audio;

void setup(){
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.begin(115200);
    SD.begin(SD_CS);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(10); // 0...21
    audio.connecttoFS(SD, "Ensoniq-ZR-76-01-Dope-77.wav");
}

void loop(){
    audio.loop();
}

// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreaminfo(const char *info){
    Serial.print("streaminfo  ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print("eof_speech  ");Serial.println(info);
}
```