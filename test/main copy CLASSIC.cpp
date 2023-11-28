
// #include <AsyncTCP.h>
// #include <WiFi.h>
#include <ESPAsyncWebServer.h>

// #include "SocketClient.h"
// #include <SocketIOclient.h>

#include "BluetoothSerial.h"

// #include "globals.h"
// const char* ssid = myssid;
// const char* password = mypassword;

// Global variables
// AsyncWebServer server(80);
// SocketClient testClient;
BluetoothSerial SerialBT;

// Send - Recieve Data
// String defineDataToSend() {
//   String stringToSend = "Hello from esp";
//   return stringToSend;
// }

// void recievedData(String data) {
//   Serial.println(data);
  // check if data is text or JSON
  // StaticJsonDocument<1024> doc;
  // DeserializationError error = deserializeJson(doc, data);
  // if (!error) {
  //   Serial.print("Got JSON: url = ");

  //   if (doc.containsKey("message") && (doc["message"], "update") == 0) {
  //     String updateURL = doc["url"];
  //     Serial.println(updateURL);
  //     // testClient.updatingMode(updateURL);
  //   }
  // } else {
  //   Serial.print(F("deserializeJson() failed: "));
  //   Serial.println(error.f_str());
  //   Serial.print("Got text: ");
  //   Serial.println(data);
  // }
// }

void setup(void) {
  Serial.begin(115200);
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // Serial.println("");

  // Wait for WiFi connection
  // Serial.print("Connecting to: ");
  // Serial.print(myssid);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("");
  // Serial.print("Connected to ");
  // Serial.println(ssid);
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  // server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  //   request->send(200, "text/plain", "Hi! This is a sample response.");
  // });

  // begin async server
  // server.begin();
  // Serial.println("HTTP server started");

  // begin bluetooth
  SerialBT.begin();
  Serial.println("Bluetooth Started! Ready to pair...");

  // test client
  // testClient.setSocketHost("sensordata.space", 443);  //192.168.0.56
  // testClient.setAppAndVersion("Development", 0.32);
  // testClient.setDeviceType("ESP32");
  // testClient.setDataToSendFunciton(defineDataToSend);
  // testClient.setRecievedDataFunciton(recievedData);
  
  // testClient.initSSL(); // if you dont want ssl use .init and change the port.
}

void loop(void) {
  // testClient.loop();
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }

  delay(20);
}