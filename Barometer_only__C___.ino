//Including libraries
#include <Wire.h>            // I2C Library
#include <Adafruit_BMP085.h> // BMP180 Library
#include <ESP8266WiFi.h>     // ESP wifi
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>       //we WebSerial print
//definations and varibles
//ESP wifi

const char *WIFI_SSID = "Antarikchya";
const char *WIFI_PASSWORD = "9869397707";
//
//const char *WIFI_SSID = "test";
//const char *WIFI_PASSWORD = "hellosatellite";
AsyncWebServer server(80);
void callback(unsigned char* data, unsigned int length)
{
  data[length] = '\0';
  WebSerial.println((char*) data);
}
//BMP180
#define seaLevelPressure_hPa 1013.25
Adafruit_BMP085 bmp;

void setup() {
  //  pinMode(A0, OUTPUT);
  pinMode(15, OUTPUT);
  // Initialize the WebSerial port.
  Serial.begin(115200); //initialize WebSerial monitor
  // Initialize I2C.
  Wire.begin();
  if (!bmp.begin()) { //Initializing BMP180
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  //connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  WebSerial.begin(&server);
  WebSerial.msgCallback(callback);
  server.begin();
}
void loop() {
  //  digitalWrite(A0, 1);
  digitalWrite(15, 1);
  
  //read BMP
  int bmp_temp =  bmp.readTemperature();
  int   pressure = bmp.readPressure();
  float my_altitude = bmp.readAltitude();
  int32_t pressure_at_sealevel = bmp.readSealevelPressure();
  int  real_altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
  
  String set = "//Including libraries
#include <Wire.h>            // I2C Library
#include <Adafruit_BMP085.h> // BMP180 Library
#include <ESP8266WiFi.h>     // ESP wifi
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>       //we WebSerial print
//definations and varibles
//ESP wifi

const char *WIFI_SSID = "Antarikchya";
const char *WIFI_PASSWORD = "9869397707";
//
//const char *WIFI_SSID = "test";
//const char *WIFI_PASSWORD = "hellosatellite";
AsyncWebServer server(80);
void callback(unsigned char* data, unsigned int length)
{
  data[length] = '\0';
  WebSerial.println((char*) data);
}
//BMP180
#define seaLevelPressure_hPa 1013.25
Adafruit_BMP085 bmp;

void setup() {
  //  pinMode(A0, OUTPUT);
  pinMode(15, OUTPUT);
  // Initialize the WebSerial port.
  Serial.begin(115200); //initialize WebSerial monitor
  // Initialize I2C.
  Wire.begin();
  if (!bmp.begin()) { //Initializing BMP180
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  //connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  WebSerial.begin(&server);
  WebSerial.msgCallback(callback);
  server.begin();
}
void loop() {
  //  digitalWrite(A0, 1);
  digitalWrite(15, 1);
  
  //read BMP
  int bmp_temp =  bmp.readTemperature();
  int   pressure = bmp.readPressure();
  float my_altitude = bmp.readAltitude();
  int32_t pressure_at_sealevel = bmp.readSealevelPressure();
  int  real_altitude = bmp.readAltitude(seaLevelPressure_hPa * 100);
  
  String set = "\n ";
  set = "\n"+ String(bmp_temp) + ", " + String(pressure) + ", " + String(my_altitude) + ", " + String(pressure_at_sealevel) + ", " + String(real_altitude);
  WebSerial.print(set);
}";
  WebSerial.print(set);
  delay(600000);
}
