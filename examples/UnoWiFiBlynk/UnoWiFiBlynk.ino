/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************/

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#include <UnoWiFiDevEdSerial1.h>
#include <BlynkSimpleWiFiLink.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "YourAuthToken";

void setup()
{
  // Debug console
  Serial.begin(115200);

  Serial1.begin(115200);
  Serial1.resetESP();
  delay(3000); //wait while WiFiLink firmware connects to WiFi with Web Panel settings
  WiFi.init(&Serial1);
  while (WiFi.status() != WL_CONNECTED) {
    delay(10);
  }
  Serial.println("You're connected to the network");

  Blynk.config(auth, BLYNK_DEFAULT_DOMAIN, BLYNK_DEFAULT_PORT);
  while(Blynk.connect() != true) {}
}

void loop()
{
  Blynk.run();
}

