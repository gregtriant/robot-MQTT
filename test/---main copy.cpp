/*
  AsyncElegantOTA Demo Example - This example will work for both ESP8266 & ESP32 microcontrollers.
  -----
  Author: Ayush Sharma ( https://github.com/ayushsharma82 )
  
  Important Notice: Star the repository on Github if you like the library! :)
  Repository Link: https://github.com/ayushsharma82/AsyncElegantOTA
*/

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <WiFi.h>
  #include <WiFiMulti.h>
#endif

#include <ESPAsyncWebServer.h>

#include "SocketClient.h"
#include <WebSocketsClient.h>
#include <SocketIOclient.h>

#include "globals.h"
const char* ssid = myssid;
const char* password = mypassword;

// Global variables
AsyncWebServer server(80);
SocketClient testClient;

// Send - Recieve Data
String defineDataToSend() {
  String stringToSend = "Hello from esp";
  return stringToSend;
}

void recievedData(String data) {
  Serial.println(data);
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
}

WiFiMulti wiFiMulti;
SocketIoclient socketIO;

#define USE_SERIAL Serial

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            USE_SERIAL.printf("[IOc] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);

            // join default namespace (no auto join in Socket.IO V3)
            socketIO.send(sIOtype_CONNECT, "/");
            break;
        case sIOtype_EVENT:
            USE_SERIAL.printf("[IOc] get event: %s\n", payload);
            break;
        case sIOtype_ACK:
            USE_SERIAL.printf("[IOc] get ack: %u\n", length);
            // hexdump(payload, length);
            break;
        case sIOtype_ERROR:
            USE_SERIAL.printf("[IOc] get error: %u\n", length);
            // hexdump(payload, length);
            break;
        case sIOtype_BINARY_EVENT:
            USE_SERIAL.printf("[IOc] get binary: %u\n", length);
            // hexdump(payload, length);
            break;
        case sIOtype_BINARY_ACK:
            USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
            // hexdump(payload, length);
            break;
    }
}


void setup(void) {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is a sample response.");
  });

  server.begin();
  Serial.println("HTTP server started");


  // USE_SERIAL.begin(921600);
    // USE_SERIAL.begin(115200);

    // //Serial.setDebugOutput(true);
    // USE_SERIAL.setDebugOutput(true);

    // USE_SERIAL.println();
    // USE_SERIAL.println();
    // USE_SERIAL.println();

    // for(uint8_t t = 4; t > 0; t--) {
    //     USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
    //     USE_SERIAL.flush();
    //     delay(1000);
    // }

    // // disable AP
    // if(WiFi.getMode() & WIFI_AP) {
    //     WiFi.softAPdisconnect(true);
    // }

    // WiFiMulti.addAP("SSID", "passpasspass");

    // //WiFi.disconnect();
    // while(WiFiMulti.run() != WL_CONNECTED) {
    //     delay(100);
    // }

    // String ip = WiFi.localIP().toString();
    // USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

    // server address, port and URL
    socketIO.begin("192.168.1.60", 3000, "/socket.io/?EIO=4"); //socket.io/?EIO=4

    // event handler
    socketIO.onEvent(socketIOEvent);
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
}