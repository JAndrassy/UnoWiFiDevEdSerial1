/*
Copyright (C) 2017 Juraj Andr�ssy
repository https://github.com/jandrassy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <UnoWiFiDevEdSerial1.h>
#include <Wire.h> // only for the IDE library detection

extern "C" {
  #include <utility/twi.h> //from Wire library
}

// Uno WiFi Developer Edition wiring
#define SC16IS750_TWI_ADDRESS 0x48
#define SC16IS750_CRYSTAL_FREQ 14745600UL

//registers
#define SC16IS750_REG_FCR 0x02
#define SC16IS750_REG_LCR 0x03
#define SC16IS750_REG_LSR 0x05
#define SC16IS750_REG_TXLVL 0x08
#define SC16IS750_REG_RXLVL 0x09
#define SC16IS750_REG_IODIR 0x0A
#define SC16IS750_REG_IOSTATE 0X0B
#define SC16IS750_REG_IOCONTROL 0x0E

//Special register set
#define SC16IS750_REG_DLL 0x00
#define SC16IS750_REG_DLH 0X01

// SC16IS750 pins for ESP8266 control
#define ESP_CH_EN_PIN 0
#define ESP_RSTB_PIN 1 // unreliable
#define ESP_GPIO0_PIN 2
#define ESP_OFF_PIN 3 // Dangerous!

byte UnoWiFiDevEdSerial1::rxBuffer[TWI_BUFFER_LENGTH];
byte UnoWiFiDevEdSerial1::rxBufferIndex = 0;
byte UnoWiFiDevEdSerial1::rxBufferLength = 0;

byte UnoWiFiDevEdSerial1::txBuffer[TWI_BUFFER_LENGTH] = {0x00}; // write register address is 0
byte UnoWiFiDevEdSerial1::txBufferLength = 1; // first byte is register address

void UnoWiFiDevEdSerial1::begin(uint32_t baud) {
  twi_init();
  if (baud > 57600) {
    twi_setFrequency(400000L);
  }
  writeRegister(SC16IS750_REG_IOCONTROL, readRegister(SC16IS750_REG_IOCONTROL) | 0x08); //UART software reset
  writeRegister(SC16IS750_REG_FCR, readRegister(SC16IS750_REG_FCR) | 0x07); // FIFO enable and clear

  uint16_t divisor = SC16IS750_CRYSTAL_FREQ / (baud * 16); // prescaler = 1
  byte lcr = readRegister(SC16IS750_REG_LCR);
  writeRegister(SC16IS750_REG_LCR, lcr | 0x80); // activate divisor registers
  writeRegister(SC16IS750_REG_DLL, (byte) divisor);
  writeRegister(SC16IS750_REG_DLH, (byte) (divisor >> 8));
  writeRegister(SC16IS750_REG_LCR, (lcr & 0xC0) | 0x03); //length 8, no parity, stop bit 1
}

void UnoWiFiDevEdSerial1::end() {
  twi_disable();
}

int UnoWiFiDevEdSerial1::available() {
  if (rxBufferIndex == rxBufferLength) {
    flush(); //complete tx
    byte length = readRegister(SC16IS750_REG_RXLVL);
    rxBufferIndex = 0;
    if (length == 0) {
      rxBufferLength = 0;
      return 0;
    }
    if (length > TWI_BUFFER_LENGTH) {
      length = TWI_BUFFER_LENGTH;
    }
    byte addr[] = {0x00};
    twi_writeTo(SC16IS750_TWI_ADDRESS, addr, 1, true, false);
    rxBufferLength = twi_readFrom(SC16IS750_TWI_ADDRESS, rxBuffer, length, true);
  }
  return rxBufferLength - rxBufferIndex;
}

int UnoWiFiDevEdSerial1::read() {
  if (!available())
    return -1;
  return rxBuffer[rxBufferIndex++];
}

int UnoWiFiDevEdSerial1::peek() {
  if (!available())
    return -1;
  return rxBuffer[rxBufferIndex];
}

size_t UnoWiFiDevEdSerial1::write(uint8_t val) {
  txBuffer[txBufferLength++] = val;
  if (txBufferLength == TWI_BUFFER_LENGTH) {
    flush();
    if (getWriteError())
      return 0;
  }
  return 1;
}

int UnoWiFiDevEdSerial1::availableForWrite() {
  return readRegister(SC16IS750_REG_TXLVL) - txBufferLength;
}

void UnoWiFiDevEdSerial1::flush() {
  if (txBufferLength == 1) // first byte is register address
    return;
  while (readRegister(SC16IS750_REG_TXLVL) < (txBufferLength - 1)); // wait for space in FIFO //TODO timeout
  byte res = twi_writeTo(SC16IS750_TWI_ADDRESS, txBuffer, txBufferLength, true, true);
  setWriteError(res);
  txBufferLength = 1; // first byte is register address
}

boolean UnoWiFiDevEdSerial1::overflow() {
  return (readRegister(SC16IS750_REG_LSR) & 0x02);
}

byte UnoWiFiDevEdSerial1::readRegister(byte regAddress) {
  byte buff[1];
  buff[0] = regAddress << 3;
  twi_writeTo(SC16IS750_TWI_ADDRESS, buff, 1, true, false);
  twi_readFrom(SC16IS750_TWI_ADDRESS, buff, 1, true);
  return buff[0];
}

void UnoWiFiDevEdSerial1::writeRegister(byte regAddress, byte value) {
  byte buff[2];
  buff[0] = (regAddress << 3);
  buff[1] = value;
  twi_writeTo(SC16IS750_TWI_ADDRESS, buff, 2, true, true);
}

void UnoWiFiDevEdSerial1::pinMode(byte pin, byte mode) {
  byte iodir = readRegister(SC16IS750_REG_IODIR);
  byte mask = (0x01 << pin);
  if (mode == OUTPUT) {
    iodir |= mask;
  } else {
    iodir &= ~mask;
  }
  writeRegister(SC16IS750_REG_IODIR, iodir);
}

void UnoWiFiDevEdSerial1::digitalWrite(byte pin, byte value) {
  byte iostate = readRegister(SC16IS750_REG_IOSTATE);
  byte mask = (0x01 << pin);
  if (value == HIGH) {
    iostate |= mask;
  } else {
    iostate &= ~mask;
  }
  writeRegister(SC16IS750_REG_IOSTATE, iostate);
}

void UnoWiFiDevEdSerial1::resetESP(boolean toBootloader) {
  if (toBootloader) {
    pinMode(ESP_GPIO0_PIN, OUTPUT);
    digitalWrite(ESP_GPIO0_PIN, LOW);
    pinMode(ESP_CH_EN_PIN, OUTPUT);
    digitalWrite(ESP_CH_EN_PIN, LOW);
    delay(5);
    pinMode(ESP_CH_EN_PIN, INPUT); // let it to pull-up resistor
    delay(50);
    pinMode(ESP_GPIO0_PIN, INPUT); // let it to pull-up resistor
  } else {
    pinMode(ESP_OFF_PIN, OUTPUT);
    digitalWrite(ESP_OFF_PIN, HIGH);
    pinMode(ESP_CH_EN_PIN, OUTPUT); // CH_EN should be activated with delay after power-up
    digitalWrite(ESP_CH_EN_PIN, LOW);
    delay(5);
    digitalWrite(ESP_OFF_PIN, LOW); // for sure
    delay(5);
    pinMode(ESP_CH_EN_PIN, INPUT); // let it to pull-up resistor
    pinMode(ESP_OFF_PIN, INPUT); // let it to pull-down resistor
  }
}

UnoWiFiDevEdSerial1 Serial1;
