#include <UnoWiFiDevEdSerial1.h>

//#define FLASHING
#define BAUD 115200L

void setup() {
  Serial.begin(BAUD);

#ifdef FLASHING
  Serial1.begin(BAUD * 2); //double speed is necessary, but it disturbs frequency test of the 'download tool'
  Serial1.resetESP(true); // reset to bootloader. no need to push the B/L button while connecting to USB
#else
  Serial1.begin(BAUD);
  Serial1.resetESP(); // power cycle
#endif
}

void loop() {
  while (Serial.available()) {
    Serial1.write(Serial.read());
  }
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
