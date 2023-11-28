
#include <Arduino.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "globals.h"
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include "I2CScanner.h"

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing

I2CScanner scanner;
Adafruit_BME680 bme; // I2C

float version = 0.2;                             // change
const char *deviceApp = "TestMQTT";
const char *deviceType = 
  #if defined(ESP32) || defined(LIBRETUYA)
  "ESP32";
  #elif defined(ESP8266)
  "ESP8266";
  #else
  "UNKNOWN";
  #endif

const char* ssid = myssid;
const char* password = mypassword;

#define MQTT_HOST "snf-889260.vm.okeanos.grnet.gr"
#define MQTT_PORT 1893

AsyncMqttClient mqttClient;

const int motor1Dir = 13;
const int motor1PWM = 12;

const int motor2Dir = 14;
const int motor2PWM = 27;

const int led = 26;
// Ticker mqttReconnectTimer;


int linear_vel = 0;
int angular_vel = 0;

int count = 0;
int dist = 0;

void sendDist() {
  // String mac = WiFi.macAddress();
  String topic = "thing.AAA.sensors.distance.sonar.front";

  DynamicJsonDocument doc(1024);
  doc["range"] = ++dist;
  char JsonToSend[1024];
  serializeJson(doc, JsonToSend);
  uint16_t packetIdPub1 = mqttClient.publish(topic.c_str(), 0, true, JsonToSend);
}

void sendIMU() {
  // String mac = WiFi.macAddress();
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Serial.print("\t\tTemperature ");
  // Serial.print(temp.temperature);
  // Serial.println(" deg C");

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tAccel X: ");
  // Serial.print(accel.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(accel.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(accel.acceleration.z);
  // Serial.println(" m/s^2 ");

  // Serial.print("\t\tMag X: ");
  // Serial.print(mag.magnetic.x);
  // Serial.print(" \tY: ");
  // Serial.print(mag.magnetic.y);
  // Serial.print(" \tZ: ");
  // Serial.print(mag.magnetic.z);
  // Serial.println(" uT");

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tGyro X: ");
  // Serial.print(gyro.gyro.x);
  // Serial.print(" \tY: ");
  // Serial.print(gyro.gyro.y);
  // Serial.print(" \tZ: ");
  // Serial.print(gyro.gyro.z);
  // Serial.println(" radians/s ");
  // Serial.println();

  String topic = "thing.AAA.sensors.motion.imu";

  DynamicJsonDocument doc(1024);
  doc["acc"]["x"] = accel.acceleration.x;
  doc["acc"]["y"] = accel.acceleration.y;
  doc["acc"]["z"] = accel.acceleration.z;

  doc["mag"]["x"] = mag.magnetic.x;
  doc["mag"]["y"] = mag.magnetic.y;
  doc["mag"]["z"] = mag.magnetic.z;

  doc["gyr"]["x"] = gyro.gyro.x;
  doc["gyr"]["y"] = gyro.gyro.y;
  doc["gyr"]["z"] = gyro.gyro.z;

  char JsonToSend[1024];
  serializeJson(doc, JsonToSend);
  uint16_t packetIdPub1 = mqttClient.publish(topic.c_str(), 0, true, JsonToSend);
}

void sendBME() {

  // String mac = WiFi.macAddress();
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  // Serial.print("Temperature = ");
  // Serial.print(bme.temperature);
  // Serial.println(" *C");

  // Serial.print("Pressure = ");
  // Serial.print(bme.pressure / 100.0);
  // Serial.println(" hPa");

  // Serial.print("Humidity = ");
  // Serial.print(bme.humidity);
  // Serial.println(" %");

  // Serial.print("Gas = ");
  // Serial.print(bme.gas_resistance / 1000.0);
  // Serial.println(" KOhms");

  // Serial.println();
  String topic = "thing.AAA.sensors.env.bme";

  DynamicJsonDocument doc(1024);
  doc["temp"] = bme.temperature;
  doc["hum"] = bme.humidity;
  doc["pres"] = bme.pressure / 100.0;
  doc["gas"] = bme.gas_resistance / 1000.0;

  char JsonToSend[1024];
  serializeJson(doc, JsonToSend);
  uint16_t packetIdPub1 = mqttClient.publish(topic.c_str(), 0, true, JsonToSend);
}

void countData() {
  // Serial.print("Last second: ");
  // Serial.println(count);
  // count = 0;
}

// Ticker timer1(countData, 1000);
Ticker timer2(sendDist, 1000);
Ticker timer3(sendIMU, 200);
Ticker timer4(sendBME, 1000);

// Ticker timer2(sendData, 10 * 1000);

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void sendConnectionInfo() {
  String mac = WiFi.macAddress();
  String localIP = WiFi.localIP().toString();
  DynamicJsonDocument doc(1024);
  doc["deviceId"] = mac;
  doc["deviceApp"] = deviceApp;
  doc["deviceType"] = deviceType;
  doc["version"] = version;
  doc["localIP"] = localIP;
  doc["protocol"] = "mqtt";
  doc["topics"] = "status";
  char JsonToSend[1024];
  serializeJson(doc, JsonToSend);

  mqttClient.publish("connection", 0, true, JsonToSend); // publish to connection topic
  Serial.println("Publishing connection");
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  sendConnectionInfo();

  uint16_t packetIdSub = mqttClient.subscribe("test.test", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);

  uint16_t packetIdSub1 = mqttClient.subscribe("pos", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub1);
  
  uint16_t packetIdSub2 = mqttClient.subscribe("thing.AAA.actuators.motion.base", 0);
  // Serial.print("Subscribing at QoS 2, packetId: ");
  // Serial.println(packetIdSub1);

  // subscribe to disconnect topic // when the server send this message the device has to disconnect and then try to connect again!
  String mac = WiFi.macAddress();
  Serial.print("MAC: ");
  Serial.println(mac);
  // char macChar[30];
  // mac.toCharArray(macChar, 30);
  // char disconnectTopic[50];
  // sprintf(disconnectTopic, "%s/disconnect", macChar);
  // uint16_t packetIdSub2 = mqttClient.subscribe(disconnectTopic, 2);
  // Serial.print("Subscribing at QoS 2, packetId: ");
  // Serial.println(packetIdSub2);
  // mqttClient.publish("test/lol", 0, true, "test 1");
  // Serial.println("Publishing at QoS 0");
  // uint16_t packetIdPub1 = mqttClient.publish("test/lol", 1, true, "test 2");
  // Serial.print("Publishing at QoS 1, packetId: ");
  // Serial.println(packetIdPub1);
  // uint16_t packetIdPub2 = mqttClient.publish("test/lol", 2, true, "test 3");
  // Serial.print("Publishing at QoS 2, packetId: ");
  // Serial.println(packetIdPub2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (reason == AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT) {
    Serial.println("Bad server fingerprint.");
  }

  // if (WiFi.isConnected()) {
  //   mqttReconnectTimer.once(2, connectToMqtt);
  // }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
  // Serial.print("  qos: ");
  // Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
}

void writeSpeed(int linear_vel, int angular_vel) {
  int w1 = linear_vel + angular_vel;
  int w2 = linear_vel - angular_vel;

  digitalWrite(motor1Dir, w2 > 0); // straight (HIGH-HIGH) // right (HIGH-LOW) // left (LOW-HIGH) // back (LOW-LOW)
  digitalWrite(motor2Dir, w1 > 0);
  
  if (w1==0 && w2==0) {
    analogWrite(motor1PWM, 0);
    analogWrite(motor2PWM, 0);
  } else {
    analogWrite(motor1PWM, map(abs(w1), 0, 150, 200, 255 ));
    analogWrite(motor2PWM, map(abs(w2), 0, 150, 200, 255 ));
  }
 
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  // Serial.print("  qos: ");
  // Serial.println(properties.qos);
  // Serial.print("  dup: ");
  // Serial.println(properties.dup);
  // Serial.print("  retain: ");
  // Serial.println(properties.retain);
  // Serial.print("  len: ");
  // Serial.println(len);
  // Serial.print("  index: ");
  // Serial.println(index);
  // Serial.print("  total: ");
  // Serial.println(total);
  // if (strcmp(topic, "pos") == 0) {
  //   // Serial.println(payload);
  //   String p(payload); 
  //   int index = p.indexOf(',');
  //   int V_lin = (p.substring(0,index)).toInt();
  //   int w = (p.substring(index+1, len)).toInt();
  //   Serial.print("Got: ");
  //   Serial.print(V_lin);
  //   Serial.print(", ");
  //   Serial.println(w);
  //   count++;
  //   if (V_lin > 150) {
  //     V_lin = 150;
  //   } else if (V_lin < -150) {
  //     V_lin = -150;
  //   }
  //   if (w > 150) {
  //     w = 150;
  //   } else if (w < -150) {
  //     w = -150;
  //   }
  //   linear_vel = V_lin;
  //   angular_vel = w;
  //   // Serial.println(p.substring(0,index));
  //   // Serial.println(p.substring(index+1, len));
  //   writeSpeed(linear_vel, angular_vel);
  // }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n");
	
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println(WiFi.localIP());

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setCredentials("porolog", "fiware"); // gregory // greg123!
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToMqtt();

  // timer1.start();
  timer2.start();
  timer3.start();
  timer4.start();

  // GPIO
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor1PWM, OUTPUT);

  pinMode(motor2Dir, OUTPUT);
  pinMode(motor2PWM, OUTPUT);

  pinMode(led, OUTPUT);

  scanner.Init();
  scanner.Scan();
	delay(500);

  // Wire.begin(0x76);
  // bme(&Wire);
  bme.begin(0x76, true);
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms


  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();
}

void loop() {
  // timer1.update();
  timer2.update();
  timer3.update();
  timer4.update();
}
