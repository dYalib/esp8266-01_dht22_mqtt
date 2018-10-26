/*
	Temperatur and humidity sensor based on ESP8266-01 microcontroller and DHT22 sensor
    Copyright (C) 2018  David Bilay

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
	along with this program. If not, see <https://www.gnu.org/licenses/>.
*/



#include <Arduino.h>
#include "DHT.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <FS.h>

#define BUILTIN_LED 1 // builtin led is connected to GPIO 1

#define DHTPIN 3               //what digital pin the DHT22 is conected to
#define DHTTYPE DHT22           //there are multiple kinds of DHT sensors, we use the DHT22 one
#define DHTMAXREADATTEMPTS 10       //max number of sensor read attempts. Goto failure state when exceeded 
#define DHTREADDELAYMS 2000       //delay after read from sensor. The DHT22 need some time...
const float tempOffset = 0;       //temperature offset to calibrate the sensor
const float humidityOffset = 0;     //humidity offset to calibrate the sensor

#define ESSID "Buebi"           // WIFI SSID 
#define WIFIPSK "****"          // WIFI passphrase (keep secret!)

#define MQTTSERVER "hassio.fritz.box" //Adress of the MQTT Server. Use a DNS name or the IPv4 Adress.
#define MQTTPORT 8883         //MQTT Server Port
#define MQTTUSER "mqtt"         //MQTT User that is allowed to connect with the server 
#define MQTTPW "***"          //MQTT User password (keep secret!)
#define MQTTCERTFILE "/lan_certificate.der" //path to cert file. This file have to be uploaded to the ESP
#define MQTTKEYFILE "/lan_privkey.der" //path to the key file. This file have to be uploaded to the ESP 
#define MQTTTOPICTEMP "ebg38/bathroom/temperature"  //The MQTT topic for the temperature 
#define MQTTTOPICHUM "ebg38/bathroom/humidity"    // The MQTT topic for the humidity

#define SLEEPTIME 600000000       //sleep time between next program execution in microsecounds




DHT dht(DHTPIN, DHTTYPE);
WiFiClientSecure wifiClient;
PubSubClient pubSubClient(wifiClient);

void ledBlink(byte code) {
  pinMode(BUILTIN_LED, OUTPUT);

  while (true) {
    digitalWrite(BUILTIN_LED, LOW); //turn the LED ON by set the voltage to LOW
    delay(1000);
    digitalWrite(BUILTIN_LED, HIGH); //turn the LED OFF by set the voltage to HIGH
    delay(1000);
    for (byte i = 0; i < code; i++) {
      digitalWrite(BUILTIN_LED, LOW); //turn the LED ON by set the voltage to LOW
      delay(200);
      digitalWrite(BUILTIN_LED, HIGH); //turn the LED OFF by set the voltage to HIGH
      delay(200);
    }
    delay(700);
  }
}


/*
     codes:
     1 - not in use
     2 - failed to connect to wifi
     3 - failed to connect to mqtt broker
     4 - failed to read from sensor
     5 . failed to load cert or key file
*/
void failure(byte code) {
  Serial.println();
  switch (code) {
    case 2 : Serial.println(F("Failed to connect to wifi!")); break;
    case 3 : Serial.println(F("Failed to connect to mqtt broker!")); break;
    case 4 : Serial.println(F("Failed to read from sensor!")); break;
    case 5 : Serial.println(F("Failed to load cert or key file!")); break;
    default: Serial.println(F("Unknown failure!")); break;
  }
  delay(500); //delay is required to print the message.
  ledBlink(code);
}


void loadCertificateAndKey() {
  SPIFFS.begin();
  File ca = SPIFFS.open(MQTTCERTFILE, "r");
  File key = SPIFFS.open(MQTTKEYFILE, "r");
  
  if (!ca or !key or !wifiClient.loadCertificate(ca) or !wifiClient.loadPrivateKey(key)) {
    failure(5);
    return;
  } else {
    Serial.println("Key and cert loaded");
  }
  
}


void connectWiFi() {

  //start connection
  Serial.println();
  Serial.print(F("MAC: "));
  Serial.println(WiFi.macAddress());
  Serial.print(F("Connecting to SSID: "));
  Serial.print(ESSID);
  WiFi.begin(ESSID, WIFIPSK);
  //wait for successfull connection
  byte i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    i++;
    if (i >60) {
      failure(2);
    }
    Serial.print(".");
    delay(500);
  }
  Serial.println("connected!");
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());
  delay(1000);
}



void connectMQTT() {
  
  loadCertificateAndKey();
  // Set the client ID
  String clientId = "ESP8266-" + String(WiFi.macAddress());
  clientId.replace(":", "");
   //configure MQTT server
  pubSubClient.setServer(MQTTSERVER, MQTTPORT);
  // Loop until we're connected
  byte i = 0;
  while (!pubSubClient.connected()) {
    i++;
    Serial.print(F("Attempting MQTT connection..."));
    if (pubSubClient.connect(clientId.c_str(), MQTTUSER, MQTTPW)) {
      Serial.println(F("connected!"));
    } else {
      if (i > 3) {
        failure(3);
      }
      Serial.print(F("failed, rc="));
      Serial.print(pubSubClient.state());
      Serial.println(F(" try again in 10 seconds"));
      // Wait 10 seconds before retrying
      delay(10000);
    }
  }
}

// Read temperature from DHT22
// isFahrenheit = true -> read temperature as Fahrenheit
// isFahrenheit = false -> read temperature as Celsius
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
float getTemperature(boolean isFahrenheit) {
  Serial.print(F("Try to read temperature from sensor..."));
  float t = 0;
  byte i = 0;
  do {
    i++;
    if (i > DHTMAXREADATTEMPTS) {
      failure(4);
    }
    t = dht.readTemperature(isFahrenheit);
    delay(DHTREADDELAYMS);
  } while (isnan(t));
  Serial.println(F("done!"));
  t += tempOffset;
  Serial.print(F("Temperature: "));
  Serial.print(t);
  Serial.println(F(" *C "));
  return t;
}


//Read humidity from DHT22
// Reading temperature or humidity takes about 250 milliseconds!
// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
float getHumidity() {
  Serial.print(F("Try to read humidity from sensor..."));
  float h = 0;
  byte i = 0;
  do {
    i++;
    if (i > DHTMAXREADATTEMPTS) {
      failure(4);
    }
    h = dht.readHumidity();
    delay(DHTREADDELAYMS);
  } while (isnan(h));
  Serial.println(F("done!"));
  h += humidityOffset;
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.println(" %\t");
  return h;
}


void setup() {
  //GPIO 3 (RX) swap the pin to a GPIO.
  pinMode(3, FUNCTION_3); 
  Serial.begin(9600);
  // Wait for serial to initialize.
  while (!Serial) { }
  Serial.println(F(""));
  Serial.println(F("-------------------------------------"));
  Serial.println(F("DEVICE STARTED!"));
  Serial.println(F("-------------------------------------"));
  //connect to WiFi
  connectWiFi();
  //connect to MQTT Server
  connectMQTT();
  //initialize dht22 sensor
  delay(2000);
  dht.begin();
}

void loop() {
  //(Re)connect WiFi, if necessary
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  //(Re)connect MQTT, if necessary
  if (!pubSubClient.connected()) {
    connectMQTT();
  }
  Serial.println(F("Publish MQTT Message"));
  pubSubClient.publish(MQTTTOPICTEMP, String(getTemperature(false), 1).c_str());
  pubSubClient.publish(MQTTTOPICHUM, String(getHumidity(), 1).c_str());
  Serial.println(F("Going to deep sleep"));
  delay(500);
  ESP.deepSleep(SLEEPTIME);
}



