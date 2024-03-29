#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

const char* ssid = "";
const char* password = "";

const uint16_t port = 10069;
const char * host = "";

#define ID 305
#define deviceType "dimmable"
#define PWM_CHANNEL 1
#define PWM_FREQ 512
#define PWM_RES 8
#define PWM_GPIO 4

#define thermometerID 307
#define deviceTypeThermometer "thermometer"
const int oneWireBus = 5;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

#define co2SensorID 503
#define deviceTypeCO2Sensor "co2sensor"

unsigned long lastUpdate;
unsigned int flip = 0;

int currentState = 0;
int percentToVal(int val) {
  return val / 100.0 * 255.0;
}

WebSocketsClient webSocket;

String RegisterPacket() {
  const size_t capacity = 2 * JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  doc["type"] = 2;

  JsonObject data = doc.createNestedObject("data");
  data["deviceId"] = ID;
  data["deviceType"] = deviceType;

  String output;
  serializeJson(doc, output);
  return output;
}

String RegisterPacketThermometer() {
  const size_t capacity = 2 * JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  doc["type"] = 2;

  JsonObject data = doc.createNestedObject("data");
  data["deviceId"] = thermometerID;
  data["deviceType"] = deviceTypeThermometer;


  String output;
  serializeJson(doc, output);
  return output;
}

String UpdatePacketThermometer(float temperature) {
  StaticJsonDocument<64> doc;

  doc["type"] = 1;

  JsonObject data = doc.createNestedObject("data");
  data["deviceId"] = thermometerID;
  data["deviceValue"] = temperature;

  String output;
  serializeJson(doc, output);
  return output;
}

String RegisterPacketCO2Sensor() {
  const size_t capacity = 2 * JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  doc["type"] = 2;

  JsonObject data = doc.createNestedObject("data");
  data["deviceId"] = co2SensorID;
  data["deviceType"] = deviceTypeCO2Sensor;


  String output;
  serializeJson(doc, output);
  return output;
}

String UpdatePacketCO2Sensor(int ppm) {
  const size_t capacity = 2 * JSON_OBJECT_SIZE(2);
  DynamicJsonDocument doc(capacity);

  doc["type"] = 1;

  JsonObject data = doc.createNestedObject("data");
  data["deviceId"] = co2SensorID;
  data["deviceValue"] = ppm;

  String output;
  serializeJson(doc, output);
  return output;
}

void handlePacket(uint8_t * payload) {
  const size_t capacity = 2 * JSON_OBJECT_SIZE(2) + 40;
  DynamicJsonDocument doc(capacity);

  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("deserializeMsgPack() failed: ");
    Serial.println(error.c_str());
    return;
  }
  int packetType = doc["type"];
  switch (packetType) {
    case 1:
      if (doc["data"]["deviceId"] == ID) {
        int value = doc["data"]["deviceValue"];
        if (value == 1) {
          currentState = 255;
        }
        else if (value == 0) {
          currentState = 0;
        }
        else currentState = percentToVal(value);
      }
      break;
    case 3:
      String result = doc["data"]["result"];
      if (!result.equals("ok")) {
        Serial.println("Register failed, restarting ...");
        ESP.restart();
      }
      break;
  }
  Serial.printf("packet type %i\n", packetType);

}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  String packet;
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED:
      Serial.printf("[WSc] Connected to url: %s\n", payload);

      packet = RegisterPacket();
      webSocket.sendTXT(packet);

      packet = RegisterPacketThermometer();
      webSocket.sendTXT(packet);

      packet = RegisterPacketCO2Sensor();
      webSocket.sendTXT(packet);
      break;
    case WStype_TEXT:
      Serial.printf("[WSc] get text: %s\n", payload);
      handlePacket(payload);
      break;
    case WStype_BIN:
      break;
    case WStype_ERROR:
      Serial.printf("[WSc] ERROR length: %u\n", length);
      break;
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }

}

void setup() {
  Serial.begin(115200);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_GPIO, PWM_CHANNEL);

  Serial.println("Booting");
  char hostname[32];
  snprintf(hostname, 32, "ESP32-%d", ID);
  WiFi.mode(WIFI_STA);
  //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  sensors.begin();
  Serial1.begin(9600, SERIAL_8N1, 25, 26);  // RX, TX

  webSocket.begin(host, port, "/", "");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  ArduinoOTA.handle();
  webSocket.loop();
  ledcWrite(PWM_CHANNEL, currentState);
  if (lastUpdate + 15 * 1000 < millis()) {
    lastUpdate = millis();
    String packet;
    if (flip == 0) {
      flip = 1;
      sensors.requestTemperatures();
      float temperatureC = sensors.getTempCByIndex(0);
      Serial.print("Temperature measured: ");
      Serial.println(temperatureC);
      packet = UpdatePacketThermometer(temperatureC);
    } else {
      flip = 0;
      int ppm = getCO2();
      Serial.print("Co2 measured: ");
      Serial.println(ppm);
      packet = UpdatePacketCO2Sensor(ppm);
    }
    webSocket.sendTXT(packet);
  }

}

byte CO2req[] = {0xFE, 0X44, 0X00, 0X08, 0X02, 0X9F, 0X25};
byte CO2out[] = {0, 0, 0, 0, 0, 0, 0};

int getCO2() {
  while (!Serial1.available())
  {
    Serial1.write(CO2req, 7);
    delay(50);
  }

  int timeout = 0;
  while (Serial1.available() < 7 )
  {
    timeout++;
    if (timeout > 10)
    {
      while (Serial1.available())
        Serial1.read();
      break;
    }
    delay(50);
  }

  for (int i = 0; i < 7; i++)
  {
    CO2out[i] = Serial1.read();
  }

  int high = CO2out[3];
  int low = CO2out[4];
  unsigned long val = high * 256 + low;
  return val * 1; // S8 = 1. K-30 3% = 3, K-33 ICB = 10
}
