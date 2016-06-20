/* Temperature Controller for Weber Smokey Mountiain using a ESP8266 and Thermocouple.
 *  
 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <String.h>
#define MAXDO   12
#define MAXCS   13
#define MAXCLK  14

\
// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
const char *ssid = "webersmoker";
const char *password = "webersmoker";

ESP8266WebServer server(80);
 
double tempC = 0.0;
double tempF = 0.0;
int tempSet = 0;
int probe1Temp = 0;
int probe2Temp = 0;
int probe3Temp = 0;
// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store last temp was read
unsigned long webUpdatePreviousMillis = 0; 
unsigned long humidifierPreviousMillis = 0;
const long webUpdateInterval = 600000;
const long humidifierUpdateInterval = 30000;
const long interval = 5000;              // interval at which to read sensor
bool isOn = false;
//root webapge
void handleRoot(){
  char temp[900];
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;
  snprintf ( temp,900,

    "<html>\
      <head>\
        <meta http-equiv='refresh' content='5'/>\
        <title>ESP8266 Demo</title>\
        <style>\
          body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; text-align: center; font-size:200%; }\
        </style>\
      </head>\
      <body>\
        <h1>Weber Smoker Web Controller</h1>\
        <p>Uptime: %02d:%02d:%02d</p>\
        <p>Temperature Celsius: %02d C </p>\
        <p>Temperature Farenheit: %02d F</p>\
        <p>Temperature Set : %02d F </p>\
        <p>Probe 1 Temp : %02d F </p>\
        <p>Probe 2 Temp : %02d F</p>\
        <p>Probe 3 Temp : %02d F</p>\
        <form action=/settings  method=POST >\
        <p>Set Temperature F\
        <input type=text name=setTemp autofocus>\
        <input type=submit value=Set></form>\
      </body>\
    </html>",

    hr, min % 60, sec % 60 , int(tempC), int(tempF) ,tempSet ,probe1Temp , probe2Temp , probe3Temp 
  );
  server.send ( 200, "text/html", temp );
}


//called when we set the temperature level
void handleSetting(){
  if (server.args() > 0 ) {
    for ( uint8_t i = 0; i < server.args(); i++ ) {
      if (server.argName(i) == "setTemp") {
        Serial.println("Arg "+ String(i)+"="+ server.arg(i));
        tempSet = server.arg(i).toInt();
        Serial.println("probe2Temp" + probe2Temp);
      }
   }
  }
  server.send(200, "text/plain", "Temperature set "  );
}

double getTemperatureC() {
  double c = thermocouple.readCelsius();
  if (isnan(c)) {
     return -1;
   } else {
     return c;
   }
}

double getTemperatureF() {
  double f = thermocouple.readFarenheit();
  if (isnan(f)) {
     return -1;
   } else {
     return f;
   }
}

void wifiConnect(){
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  // Connect to WiFi network
  WiFi.begin(ssid, password);
  Serial.print("\n\r \n\rWorking to connect");
 
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  Serial.print(".");
  }
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println();
  Serial.print("Configuring access point...");
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
   
  server.on ( "/", handleRoot );
  server.on ( "/settings", handleSetting);
  server.begin();
  Serial.println("HTTP server started");
}
 
void loop(void)
{
  tempC = getTemperatureC();
  tempF = getTemperatureF();
  server.handleClient();

  //updateThingSpeak("1=" + String((int)humidity) + "&2=" + String((int)tempC) + "&3=" + String(humidifierStatus) + "&4=" + String(probe1Temp) + "&5=" + String(probe2Temp));
} 
 

