# Uno WiFi (Developer Edition) Serial1

Arduino Uno WiFi is an Arduino UNO R3 with ESP8266 integrated on the board. It was developed and manufactured by Arduino.org.

Uno WiFi Developer Edition connects ATmega328 to ESP8266 using additional on board UART chip SC16IS750 ([here as module](http://sandboxelectronics.com/?product=sc16is750-i2cspi-to-uart-bridge-module)). This additional UART is connected to ATmega as I2C device.

This library creates a Serial1 object over SC16IS750 on Arduino Uno WiFi Developer Edition. This Serial1 enables to communicate with the on-board ESP8266 over it's serial interface. The included tool EspProxy enables accessing the on-board ESP8266 over USB for 'flashing' tools, IDE sketch upload or Serial Monitor.

## Install this library

Download the contents of this GitHub repository as ZIP using the green 'Clone or download' button. Extract the ZIP and copy the folder UnoWiFiDevEdSerial1-master into your Arduino libraries folder (a subfolder in the folder where sketches are saved by IDE). Rename the folder to UnoWiFiDevEdSerial1 (delete -master). Reopen the IDE.

## Uno WiFi with Espressif AT firmware

### Prepare

Download the AT firmware from [Espressif download page](http://espressif.com/en/support/download/at?keys=&field_type_tid%5B%5D=14) and unzip it. Replace the esp_init_data_default.bin with [this one](doc/esp_init_data_UnoWiFi.bin). It has the 40MHz crystal setting. 

1. Open in IDE the EspProxy.ino from UnoWiFiDevEdSerial1 examples.
2. Uncomment the #define FLASHING line (remove the // at the beginning of the line) 
3. Upload the EspProxy sketch into UnoWiFi. (No need to save it.)
4. let the sketch open in IDE

### Option 1 - flashing with esptool.py

Install Python 2.7 and [esptool](https://github.com/espressif/esptool).

Go on command line in the folder with the AT firmware files and run the following command with the COM port of your Uno WiFi:

`esptool.py -p COM-PORT write_flash -ff 80m -fm qio -fs 4MB 0x0 boot_v1.7.bin_rep 0x01000 at/512+512/user1.1024.new.2.bin 0x3fc000 esp_init_data_default.bin 0xfe000 blank.bin 0x3fe000 blank.bin`

### Option 2 - flashing with Espressif Flash Download Tools (Windows)

Download the tool from [Espressif download page](http://espressif.com/en/support/download/other-tools), unzip it and run.

Select files, addresses and settings like on this picture:

![settings](doc/EspFDTUnoWiFi.JPG)

Use the Start button to execute.

Note: In field DETECTED INFO the detected frequency will be half of the real frequency. That is why the SpiAutoSet must not be checked.

### Cheking the firmware

1. Return to EspProxy.ino sketch in the IDE
2. Comment the #define FLASHING line (make Undo or put // at the beginning of the line) 
3. Upload the EspProxy sketch into UnoWiFi.

Open the Serial Monitor. EspProxy sketch resets the Esp and you will see the boot log at 115200 baud. The strange characters sequence is normal.

Set the CR/LF setting and send test command 'AT'. The ESP should replay with OK.

![Serial Monitor](doc/SerMonEspAT.JPG)

Next commands:
* AT+CWMODE=1 sets tha STA mode
* AT+CWJAP="ssid","pass" connects to AP
* AT+CIUPDATE updates the firmware to the latest version
* AT+GMR - prints the version

### Example sketch with WiFiEsp library

Install the [WiFiEsp library](https://github.com/bportaluri/WiFiEsp).

Open the WiFiEspWebClient example. Change the WiFi credentials and upload the sketch.

WiFiEsp library has timeout issues. One of them causes buffer overflow with Uno WiFi Serial1. Instructions are in the sketch.

## Uno WiFi with WiFi Link firmware

## Serial1 overflow

## I2C a.k.a. Wire a.k.a. TWI


