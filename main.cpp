#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <WiFiClient.h>

#include <list>

#define LED_BUILTIN 13
#define SERIAL_PORT 115200

Adafruit_BME280 bme;
AsyncWebServer server(80);

int tempThreshold = 9999;
int humThreshold = 9999;
int prsThreshold = 9999;

WiFiUDP udp;
const uint16_t serverPort = 8080;
const char* serverIp = "192.168.1.61"; // Set according to server
String sensorIp;

std::list<float *> sensorData;
uint32_t delayTime = 10000;

// ####################### SERVER #######################
void writeJsonResponse(AsyncJsonResponse *response) {
    JsonObject& root = response-> getRoot();

    root["id"] = sensorIp;
    JsonArray& array = root.createNestedArray("data");

    for (auto itr = sensorData.begin(); itr != sensorData.end();) {
        float *line = (float *) *itr;

        JsonObject& data = array.createNestedObject();
        data["time"] = line[0];
        data["temp"] = line[1];
        data["hum"] = line[2];
        data["press"] = line[3];

        itr = sensorData.erase(itr);
    }
}

void setupServer() {
    // Flushes all collected data 
    server.on("/flush", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncJsonResponse *response = new AsyncJsonResponse();

        writeJsonResponse(response);

        response -> setLength();
        request->send(response);
    });

      // Sets mesurement interval
    server.on("/interval" , HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebParameter* p = request->getParam(0);
        int newInterval = p ->value().toInt();
        delayTime = newInterval;
        request->send(200);
    });

    // Sets the warning values
    server.on("/warning", HTTP_GET, [](AsyncWebServerRequest *request) {
        int params = request -> params();
        for(int i= 0; i < params; i++) {
            AsyncWebParameter* p = request->getParam(i);
            String param = p ->name();

            if (param.equals("temp")) 
            {
                tempThreshold = p->value().toInt();    
            } 
            if (param.equals("hum")) 
            {
                humThreshold = p->value().toInt();
            } 
            if (param.equals("pres")) 
            {
                prsThreshold = p->value().toInt();
            }
        }
        request->send(200);
    });

    //Mesures and receives the tmp/hum/pres
    server.on("/gettemperature", HTTP_GET, [](AsyncWebServerRequest *request) {
      float temperature = bme.readTemperature();
      String json = "{\"temperature\":";
      json += String(temperature);
      json += "}";
      request->send(200, "application/json", json);
    });

    server.on("/getpressure", HTTP_GET, [](AsyncWebServerRequest *request) {
      float temperature = bme.readPressure();
      String json = "{\"pressure\":";
      json += String(temperature);
      json += "}";
      request->send(200, "application/json", json);
    });

    server.on("/gethumidity", HTTP_GET, [](AsyncWebServerRequest *request) {
      float humidity = bme.readHumidity();
      String json = "{\"humidity\":";
      json += String(humidity);
      json += "}";
      request->send(200, "application/json", json);
    });

    // Route to set GPIO to LOW
    server.on("/off", HTTP_GET, [](AsyncWebServerRequest *request) {
        digitalWrite(LED_BUILTIN, LOW);
        request->send(200);
    });

    // Route to set GPIO to HIGH
    server.on("/on", HTTP_GET, [](AsyncWebServerRequest *request) {
        digitalWrite(LED_BUILTIN, HIGH);
        request->send(200);
    });

    server.begin();
}

// ####################### CLIENT #######################

void registerNode() {
    String registration = "{\"type\": \"registration\", \"ip\": \"" + sensorIp + "\"}";
    udp.beginPacket(serverIp,serverPort);
    udp.println(registration);
    udp.endPacket();
}

void sendToServer(String type, float value) {
    String warning = "{\"type\": \"warning\", \"warning\": Danger! The " + type + " is " + value + "}";
    udp.beginPacket(serverIp,serverPort);
    udp.println(warning);
    udp.endPacket();
}

void warningAlerts(float *arr) {
    String type;
    float value;
    if (arr[1] > tempThreshold) {
        type = "temperature";
        value = arr[1];
        sendToServer(type, value);
    }
    else if (arr[2] > humThreshold ) {
        type = "humidity";
        value = arr[1];
        sendToServer(type, value);
    } 
    else if (arr[3] > prsThreshold) {
        type = "temperature";
        value = arr[1];
        sendToServer(type, value);
    }
}

// ####################### SENSING #######################
void setupSensor() {
    Serial.println("Connecting sensor");

    bool status = bme.begin(0x76);


    if (!status) {
        Serial.println("Cant't find sensor. Check wiring!");
        while (1);
    }
}

void sensingStuff(std::list<float *> &list) {
    // TODO - the time!
    float time = millis();
    float tmp = bme.readTemperature();
    float hum = bme.readHumidity();
    float prs = bme.readPressure();

    float *arr = new float[4]{time, tmp, hum, prs};
    list.push_back(arr);

    // An alert will be triggered if the value is higher than the threshold
    warningAlerts(arr);

    // Loging 
    Serial.print("temp = ");
    Serial.println(tmp);

    Serial.print("hum = ");
    Serial.println(hum);

    Serial.print("pressure = ");
    Serial.println(prs);
}

// ####################### SETUP #######################

void turnOnLed() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void startSerial() {
    Serial.begin(SERIAL_PORT);
}


// ####################### WIFI #######################
void hardcoded() {
    const char* ssid     = "Bbox-24C32E50";
    const char* password = "Wifilucas64990";  
    WiFi.begin(ssid, password);
}

void smartConfig() {
    WiFi.mode(WIFI_STA);
    WiFi.beginSmartConfig();
    Serial.println("Waiting for SmartConfig");
    
    while (!WiFi.smartConfigDone()) {
        delay(1000);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("SmartConfig Done");
    Serial.println("Waiting for WiFi");
}

// I can change this to loop through all the available networks
void setupAp() {
    // SMART CONFIG
    //smartConfig();

    // HARDCODED
    hardcoded();

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }

    Serial.println("");

    Serial.println("WiFi Connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    sensorIp = WiFi.localIP().toString();
}

// ####################### MAIN #######################

void setup() {
    startSerial();
    turnOnLed();
    setupSensor();
    setupAp();
    registerNode();
    setupServer();
}

void loop() {
    sensingStuff(sensorData);
    delay(delayTime);
}
