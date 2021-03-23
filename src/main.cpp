#include <Arduino.h>
//#include "BluetoothSerial.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif

//define DHT11 sensor pin
#define DHTPIN 23
#define DHTTYPE DHT11

// Replace the next variables with your SSID/Password combination
const char* ssid = "Surface";
const char* password = "Guckste192";
const char* mqtt_server = "192.168.137.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

DHT dht(DHTPIN, DHTTYPE);


//BluetoothSerial SerialBT;


void setup_wifi() {
  delay(10);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  //do nothing right now
}

void setup() {
  Serial.begin(115200);

    //start communication with dht11 sensor
  dht.begin();

  //SerialBT.begin("ESP32test"); //Bluetooth device name

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {


if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 30000) {
    lastMsg = now;

    float temp = dht.readTemperature();
    float humi = dht.readHumidity();

    char tempString[8];
    char humString[8];
    dtostrf(temp,1,2,tempString);
    dtostrf(humi,1,2,humString);

    client.publish("temperature", tempString);
    client.publish("humidity", humString);

    //SerialBT.println(temp);
    //SerialBT.println(humi);
  }
}
