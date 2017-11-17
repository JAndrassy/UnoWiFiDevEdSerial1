/*
 WiFiLink example: WebClient
 https://github.com/jandrassy/arduino-library-wifilink
 */

#include <UnoWiFiDevEdSerial1.h>
#include "WiFiLink.h"

char server[] = "arduino.cc";
boolean sendRequest = true;

WiFiClient client;

void setup() {
  Serial.begin(115200);

  Serial1.begin(115200);
  Serial1.resetESP();
  delay(3000); //wait while WiFiLink firmware connects to WiFi with Web Panel settings

  WiFi.init(&Serial1);
  while (WiFi.status() != WL_CONNECTED) {
    delay(10);
  }
  Serial.println("You're connected to the network");
  printWifiStatus();
  Serial.println();
}

void loop() {
  if (sendRequest) {
    Serial.println("Starting connection to server...");
    if (client.connect(server, 80)) {
      Serial.println("Connected to server");
      client.println("GET /asciilogo.txt HTTP/1.1");
      client.println("Host: arduino.cc");
      client.println("Connection: close");
      client.println();
    }
    sendRequest = false;
  }

  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }

  if (!client.connected()) {
    Serial.println();
    Serial.println("Disconnecting from server...");
    client.stop();

    while (true)
      ;
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
