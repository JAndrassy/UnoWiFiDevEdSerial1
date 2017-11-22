/*
Copyright (C) 2017 Juraj Andrássy
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

#ifndef _UNOWIFIDEVEDSERIAL1_H_
#define _UNOWIFIDEVEDSERIAL1_H_

#include "Arduino.h"

class UnoWiFiDevEdSerial1 : public Stream {

public:
  void begin(uint32_t baud); //inits twi and SC16IS750
  void end(); // disables twi

  // Stream implementation
  int read();
  int available();
  int peek();

  // Print implementation
  size_t write(uint8_t val);
  int availableForWrite(void);
  void flush();
  using Print::write; // pull in write(str) and write(buf, size) from Print

  // special functions
  boolean overflow(); // check SC16IS750 rx overflow flag
  void resetESP(boolean toBootloader = false); // ESP control

private:
  static byte rxBuffer[];
  static byte rxBufferIndex;
  static byte rxBufferLength;

  static byte txBuffer[];
  static byte txBufferLength;

  // SC16IS750 functions
  byte readRegister(byte regAddress);
  void writeRegister(byte regAddress, byte value);
  void pinMode(byte pin, byte mode);
  void digitalWrite(byte pin, byte value);

public:
  // virtual destructor to get rid of compilation warning
  virtual ~UnoWiFiDevEdSerial1() {};

};

extern UnoWiFiDevEdSerial1 Serial1;
#define HAVE_HWSERIAL1

#endif
