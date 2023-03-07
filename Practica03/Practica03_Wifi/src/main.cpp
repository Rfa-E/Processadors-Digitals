#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <Arduino.h>

// SSID & Password
const char* ssid = "MiFibra"; // Enter your SSID here
const char* password = "********"; //Enter your Password here

AsyncWebServer server(80); // Object of WebServer(HTTP port, 80 is defult)

void setup(){
  Serial.begin(115200);

  if(!SPIFFS.begin())
  {
     Serial.println("An Error has occurred while mounting SPIFFS");
     return;
  }

  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  
  Serial.println("");
  Serial.println("WiFi connected successfully");
  Serial.print("Got IP: ");
  Serial.println(WiFi.localIP()); //Show ESP32 IP on serial
  server.on("/Sharingan", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/Sharingan.html", "text/html");
  });
  server.on("/Sharingan.css", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(SPIFFS, "/Sharingan.css", "text/css");
  });

  server.begin();
  Serial.println("HTTP server started");
  delay(100);
}

void loop() 
{

}
