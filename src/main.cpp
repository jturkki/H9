/*
* Measures temperature, humidity and airpressure using BME280 sensor
* and luminosity using TSL2591 sensor with five minute intervals.
* Reads the outside temperature in Mikkeli from openweathermap.org.
* Sends all the data to asksensors.com to two separate sensors.
*
* Senses if sensor is moved and sends movement notice (1) to server
* and lights red led when moved (sends notice only once, until
* acknowledged. After acknowledgement is ready to send another).
* Pressing the button acknowledges the movement and sends notice (0)
* to server and turns off the red led. 
*
* TIES536 Sulautettu Internet
* Harjoitus 9
* Jyri Turkki
*/


/*
 Based on: 
 Repeating WiFi Web Client

 This sketch connects to a a web server and makes a request
 using a WiFi equipped Arduino board.

 created 23 April 2012
 modified 31 May 2012
 by Tom Igoe
 modified 13 Jan 2014
 by Federico Vanzati
 modified 1 Nov 2022
 by Jukka Ihalainen
 modified 5 Nov 2022
 by Jyri Turkki
 modified 16 Nov 2022
 by Jyri Turkki

 http://www.arduino.cc/en/Tutorial/WifiWebClientRepeating
 This code is in the public domain.
 */
 
#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include "Adafruit_TSL2591.h"
#include <ArduinoJson.h>
#include <Arduino_LSM6DS3.h>

// Introducing used functions (functions in the same order as in code)
void httpRequest();
void httpRequest2(int a);
void sendingToServer(String url);
void mittaa();
void printWifiStatus();
void displayTSL2591SensorDetails();
void configureTSL2591Sensor();
void haeJson();
void tarkistaLiike();
void tarkistaButton();

const int buttonPin = 7;                // the number of the pushbutton Pin
const int ledPinRed = 5;                // the number of the red led Pin


char ssid[] = "";        // network SSID (name) 
char pass[] = "";                       // network password 
char apikey1[] = "";   //apikey of first sensor
char apikey2[] = "";   //apikey of second sensor
int keyIndex = 0;                       // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;

// Initialize the WiFi client library, BME280 and TSL2591
WiFiClient client;
Adafruit_BME280 bme;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// server address where data is sent:
char server[] = "asksensors.com";

volatile int temp, pressure, humidity, luminosity;      // sensor measured values
unsigned long lastConnectionTime = 0;                   // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 5L * 60L * 1000L; // delay (5 min) between updates, in milliseconds
volatile int jsonOutsideTemp;                           // outside temperature in Mikkeli
volatile int noticeSent = 0;                            // notice sent to server (1), acknowledging sent to server (0) 


//Json data server and client
char jsonServer[] = "api.openweathermap.org";
WiFiClient jsonClient;


void setup() {

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPinRed, OUTPUT);

  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; 
  }

  Serial.println(F("BME280 test"));
  
  unsigned statusBme = bme.begin(0x76);    // starting BME280 and testing if found

  if (!statusBme)  {
    Serial.println("Could not find a valid BME280 sensor");
    Serial.print("SensorID was: 0x"); Serial.println("bme.sensorID(), 16");

    while(1) delay(10);
  }
  Serial.println("Found BME280 sensor");
  
  
  Serial.println(F("Starting Adafruit TSL2591 Test!"));    // starting TSL2591 and testing if found
  
  if (tsl.begin()) {
    Serial.println(F("Found a TSL2591 sensor"));
  }  else {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1);
  }


  displayTSL2591SensorDetails();      // show the TSL2591 info
  configureTSL2591Sensor();           // set gain and integration time for TSL2591 sensor


  
  if (!IMU.begin()) {                 // starting LSM6DS3 accelerometer
    Serial.println("Failed to initialize IMU!");
    while (1);
  }


  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");


  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  // you're connected now, so print out the status:
  printWifiStatus();
}



void loop() {

  // if there's incoming data from the net connection.
  // send it out the serial port.  This is for debugging
  // purposes only:
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  tarkistaLiike();         // if sensor is moved, turn on red led and send notice to server
 
  tarkistaButton();        // if button is pressed, turn off red led and send acknowledgement to server

  
  //After 5 min, get sensor readings and connect to server:
  if ((millis() - lastConnectionTime) > postingInterval) {

    mittaa();       // read BME280 and TSL2591
    haeJson();      // read outside temp in Mikkeli from openweathermap.org
    Serial.print("Mikkeli: "); Serial.println(jsonOutsideTemp);         // send outside temp to serial, for debugging purposes only

    httpRequest();  // send sensor data and outsidetemp to asksensors.com
  }
}
 


//	This method makes a HTTP connection to the server. It sends the set interval (5min) sensor values and weather info.
void httpRequest() 
{
  // close any connection before send a new request.
  // This will free the socket on the NINA module
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("connecting...");

    String url = "https://asksensors.com/api.asksensors/write/"; //For sensor one
    url += apikey1;
    url += "?module1="; //Pressure: line
    url += pressure;
    url += "&module2="; //Humidity: line
    url += humidity;
    url += "&module3="; //Temperature: line
    url += temp;
    url += "&module4="; //Temperature: table
    url += temp;
    
    sendingToServer(url);  // Sending sensor one data

    url = "https://asksensors.com/api.asksensors/write/"; //For sensor two
    url += apikey2;
    url += "?module1=";   //Luminosity: line
    url += luminosity;
    url += "&module2=";   //Outside temp in Mikkeli: line
    url += jsonOutsideTemp;

    sendingToServer(url); // Sending sensor two data
    
    // note the time that the connection was made:
    lastConnectionTime = millis();
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}



void httpRequest2(int a) {             // sending movement status to server (a=1 movement, a=0 acknowledge)
  // close any connection before send a new request.
  // This will free the socket on the NINA module
  client.stop();

  // if there's a successful connection:
  if (client.connect(server, 80)) {
    Serial.println("connecting...");

    String url = "https://asksensors.com/api.asksensors/write/"; //
    url += apikey2;
    url += "?module3="; //Movement: Binary
    url += a;
    url += "&module4="; //Movement: Table
    url += a;

    sendingToServer(url);

  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
}



void sendingToServer(String url)      // sending data to server with given url
{
    // send the HTTP GET request:
    Serial.print("********** requesting URL: ");
      Serial.println(url);
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + server + "\r\n");
    client.println();
}


void mittaa()         // read BME280 sensor for temperature, pressure and humidity
// and TSL2591 sensor for luminosity
{               
  temp = bme.readTemperature();
  pressure = ( bme.readPressure() / 100.0F);
  humidity = bme.readHumidity();
  luminosity = tsl.getLuminosity(TSL2591_VISIBLE);  
}



void printWifiStatus() 
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}


void displayTSL2591SensorDetails(void)       // print the TSL2591 details
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}


void configureTSL2591Sensor()        // set gain and integration time for TSL2591
// You can change the gain on the fly, to adapt to brighter/dimmer light situations
// Set LOW (1x gain) for bright, MED (25x gain) for normal and HIGH (428x gain) for dim light

// Changing the integration time gives you a longer time over which to sense light
// longer timelines are slower, but are good in very low light situtations!
// For bright light 100MS (shortest), for dim light 600MS (longest). Use 100MS intervals.
{
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
}



void haeJson()           // Get Json data into a variable
{
 Serial.println("\nStarting connection to JSONserver...");

  // if you get a connection, report back via serial:
  if (jsonClient.connect(jsonServer, 80)) 
  {
    Serial.println("connected to server");
    
    // Send HTTP request:
    jsonClient.println(F("GET /data/2.5/weather?q=Mikkeli&appid=aa65788a0c7617082906de01f740ad35&units=metric HTTP/1.0")); 
    jsonClient.println(F("Host: openweathermap.org"));
    jsonClient.println(F("Connection: close"));
    if (jsonClient.println() == 0) {
      Serial.println(F("Failed to send request"));
      jsonClient.stop();
      return;
    }
    
    // Skip HTTP headers
    char endOfHeaders[] = "\r\n\r\n";
    if (!jsonClient.find(endOfHeaders)) {
      Serial.println(F("Invalid response"));
      jsonClient.stop();
      return;
    }

    // Allocate the JSON document
    // Use https://arduinojson.org/v6/assistant to compute the capacity.
    const size_t capacity = 1024;  // calculated with assistant
  
    DynamicJsonDocument doc(capacity);

    // Parse JSON object
    DeserializationError error = deserializeJson(doc, jsonClient);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      jsonClient.stop();
      return;
    }

    // Extract values

  jsonOutsideTemp = doc["main"]["temp"];

  // Disconnect
  jsonClient.stop();
  }
}



void tarkistaLiike() {                 // if sensor is moved more than treshold for effect, turn on red led and send notice (1) to server

  float x, y, z, g;                    //x, y and z are acceleration values of respective axes and g is their combined effect

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);     // read acceleration values from LSM6DS3 sensor

    g = sqrt(x*x+y*y+z*z);             // calculates the combined effect of acceleration (Pythagora's clause)

    // if effect more than 1.2, turn on red led and send notice (1) to server if not sent already
    if((g>1.2) && (noticeSent == 0)){                          
      
      digitalWrite(ledPinRed, HIGH);   
      httpRequest2(1);                 // notice to server for movement (1) 
      noticeSent = 1;                                              
    }
  }
}


void tarkistaButton() {                // if button is pressed, turn off red led and send acknowledgement to server (0)
  
  if (digitalRead(buttonPin) == LOW) {
    digitalWrite(ledPinRed, LOW);
    httpRequest2(0);                  // sending acknowledgement of movement (0) 
    noticeSent = 0;                   // sensor will send a notice to server again if moved
  }
}
