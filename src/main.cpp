/*
MQTT Example for ESP8266, how to publish and subcribe to Mqtt Topic

Board: ESP12E, ESP12F, NodeMCU 86288
Library Used:
  - beegee-tokyo/DHT sensor library for ESPx@^1.18
	- 256dpi/MQTT@^2.5.0
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DHTesp.h>
#include <MQTT.h>

#define WIFI_SSID "Steff-IoT"
#define WIFI_PASSWORD "steffiot123"

#define MQTT_BROKER "54.255.200.207"
#define MAX_CHANNEL 4
#define PIN_DHT11 0

const uint8 g_arLedPins[MAX_CHANNEL] = {14, 12, 13, 15};
void InitLeds();
DHTesp dht;
bool readDhtSensor(float& temp, float& humidity);

void WiFi_Connect();

void Mqtt_Connect();
void onMqttMessageReceived(String &topic, String &payload);
WiFiClient wifiClient;
MQTTClient mqtt;
String g_strMqttTopicPrefix;

void setup() {  
  Serial.begin(115200);
  InitLeds();
  Serial.println("Booting...");
  Serial.printf("ESP ChipID: %08X\n", ESP.getChipId());
  dht.setup(PIN_DHT11, DHTesp::DHT11);
  WiFi_Connect();

  // set mqtt broker
  mqtt.begin(MQTT_BROKER, wifiClient);
  // mqtt.onMessage(onMqttMessageReceived);
  mqtt.onMessage([&](String &topic, String &payload){
    Serial.println("mqtt receive[" + topic + "]: " + payload);
    if (topic==(g_strMqttTopicPrefix+"/leds") && payload.length()>=3)
    {
      int nLed   = payload.charAt(0)-'1';
      int nValue = payload.charAt(2)-'0';
      if ((nLed>=0 && nLed<=3) && (nValue>=0 && nValue<=1))
        digitalWrite(g_arLedPins[nLed], nValue);
    }
  });
  Mqtt_Connect();
}

void loop() {
  static int nCount = 0;
  float temp, hum;

  mqtt.loop();
  delay(10);  // <- fixes some issues with WiFi stability

  if (!mqtt.connected()) 
    Mqtt_Connect();

  if (nCount++>=5) // sent every 5 seconds
  {
    nCount = 0;
    if (readDhtSensor(temp, hum))
    {
      Serial.printf("Temperature: %.2f, Humidity: %.2f\n", temp, hum);
      mqtt.publish(g_strMqttTopicPrefix+"/temperature", String(temp));
      mqtt.publish(g_strMqttTopicPrefix+"/humidity", String(hum));
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(900);
}

void InitLeds()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // active LOW-> LOW = Led ON
  for (int i=0;i<MAX_CHANNEL; i++)  
  {
    pinMode(g_arLedPins[i], OUTPUT);
    digitalWrite(g_arLedPins[i], LOW);
  }
}

void WiFi_Connect()
{
  // connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wifi ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected with IP address: "+WiFi.localIP().toString());
}

void Mqtt_Connect()
{
  char szMqttClientId[32];
  sprintf(szMqttClientId, "esp8622_%08X", ESP.getChipId());
  Serial.print("Connecting to mqtt broker ");
  while (!mqtt.connect(szMqttClientId)) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");

  g_strMqttTopicPrefix = szMqttClientId;
  mqtt.subscribe(g_strMqttTopicPrefix+"/leds");
}

void onMqttMessageReceived(String &topic, String &payload)
{
  Serial.println("incoming: " + topic + " - " + payload);
}

bool readDhtSensor(float& temp, float& humidity)
{
  temp = dht.getTemperature();
  humidity = dht.getHumidity();
  return dht.getStatus()==DHTesp::ERROR_NONE;
}