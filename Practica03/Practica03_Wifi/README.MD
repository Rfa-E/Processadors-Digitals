# Practica 3.1. Conexion Wifi
## **Introducción**
En esta práctica se realiza una conexión wifi en la ESP32 para la cual, tendremos que generar una página web, mediante un web server. 

(No se requiere de montaje).
## **Software y su funcionamiento**
### **- Cabecera del código**
Se incluyen las librerías necesarias para poder utilizar el wifi y generar el web server. También, se crean dos variables constantes para introducir los datos necesarios del wifi, como su SSID y el password. A continuación, se define el puerto HTTP que utilizara el Web server, que lo dejaremos por defecto.
```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
// SSID & Password
const char* ssid = "*****"; // Enter your SSID here
const char* password = "*****"; //Enter your Password here
WebServer server(80); // Object of WebServer(HTTP port, 80 is defult)
```
### **- Estructura del Setup**
Se establece la velocidad de comunicación del dispositivo en 115200, y redactamos el conjunto de funciones para realizar la conexión y visualizar el estado de esta.
```cpp
void setup() 
{
  Serial.begin(115200);
  Serial.println("Try Connecting to ");
  Serial.println(ssid);
// Connect to your wi-fi modem
  WiFi.begin(ssid, password);
// Check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP()); //Show ESP32 IP on serial
  server.on("/", handle_root);
  server.begin();
  Serial.println("HTTP server started");
  delay(100);
} 
```
### **- Estructura del Loop**
Solamente se escribirá la función "handleClient()" que servirá para recibir las peticiones del servidor. 
```cpp
void loop() 
{
server.handleClient();
} 
```
### **- Estructura HTML**
Finalmente, se introduce el código en Html o css que se tendrá que visualizar en el web server. Sintaxis de ejemplo:
```cpp
// HTML & CSS contents which display on web server
String HTML = "<!DOCTYPE html>\
<html>\
<body>\
<h1>My Primera Pagina con ESP32 - Station Mode &#128522;</h1>\
</body>\
</html>";
// Handle root url (/)
void handle_root() 
{
  server.send(200, "text/html", HTML);
}
```
## **código completo**
```cpp
#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>

// SSID & Password
const char* ssid = "MiFibra-2C06"; // Enter your SSID here
const char* password = "*********"; //Enter your Password here

void handle_root(void);

WebServer server(80); // Object of WebServer(HTTP port, 80 is defult)

void setup() 
{
  Serial.begin(115200);
  Serial.println("Try Connecting to ");
  Serial.println(ssid);
// Connect to your wi-fi modem
  WiFi.begin(ssid, password);
// Check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP()); //Show ESP32 IP on serial
  server.on("/", handle_root);
  server.begin();
  Serial.println("HTTP server started");
  delay(100);
}

void loop() 
{
  server.handleClient();
}
// HTML & CSS contents which display on web server
String HTML = "<!DOCTYPE html>
  <head>
    <link type="text/css" rel="stylesheet" href="include/Sharingan.css"/>
  </head>
  <body>
    <html>
            <div class="sharingan">
                    <div class="lineaInterior">
                        <div class="circulo" style="transform: rotate(-30deg)">
                            <div class="triangulo">
                            </div>
                        </div>

                        <div class="circulo" style="left:130px; top: 50px; transform: rotate(80deg)">
                            <div class="triangulo">
                            </div>
                        </div>

                        <div class="circulo" style="left: 50px; top: 130px; transform: rotate(180deg)">
                            <div class="triangulo">
                            </div>
                        </div>

                     </div>
                     <div class="lineaInterior" style="width:40%; height: 40%; background-color: black">
                </div>
            </div>
            </html>
  </body>
</html>";
// Handle root url (/)
void handle_root() 
{
  server.send(200, "text/html", HTML);
}
```