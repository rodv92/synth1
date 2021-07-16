/*
 * DigiPotX9Cxxx.h - Arduino library for managing digital potentiometers X9Cxxx (xxx = 102,103,104,503).
 * By Timo Fager, Jul 29, 2011.
 * Released to public domain.
 **/

#ifndef DigiPotX9Cxxx_h
#define DigiPotX9Cxxx_h

#include "Arduino.h"
#include "Adafruit_MCP23017.h"
//#include "/home/rod/Arduino/libraries/Adafruit_MCP23017_Arduino_Library/Adafruit_MCP23017.cpp"

#define DIGIPOT_UP   HIGH
#define DIGIPOT_DOWN LOW
#define DIGIPOT_MAX_AMOUNT 99
#define DIGIPOT_UNKNOWN 255
#define MCP_UNUSED 255

class DigiPot
{
 public:
  DigiPot(uint8_t incPin, uint8_t udPin, uint8_t csPin);
  DigiPot(uint8_t incPin, uint8_t udPin, uint8_t csPin, uint8_t addrMCP, Adafruit_MCP23017 mcp);
  void begin();
  void beginMCP();
  void increase(uint8_t amount);
  void decrease(uint8_t amount);
  void change(uint8_t direction, uint8_t amount);
  void set(uint8_t value);
  uint8_t get();
  void reset();

 private:
  Adafruit_MCP23017 _mcp;
  uint8_t _i;
  uint8_t _incPin;
  uint8_t _udPin;
  uint8_t _csPin;
  uint8_t _addrMCP;
  uint8_t _currentValue;
};

#endif
