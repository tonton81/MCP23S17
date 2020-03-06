#if !defined(_MCP23S17_H_)
#define _MCP23S17_H_

#include "Arduino.h"

typedef void (*mcp23s17_class_ptr)();

typedef enum MCP23S17_DEV_TABLE {
  MCP23S171 = (uint32_t)0x400B8000,
  MCP23S172 = (uint32_t)0x400D0000,
  MCP23S173 = (uint32_t)0x400BC000,
  EWM  = (uint32_t)0x400B4000
} MCP23S17_DEV_TABLE;

#define MCP23S17_CLASS template<SPIClass* bus, uint8_t chipSelect, uint32_t speed = 2000000>
#define MCP23S17_FUNC template<SPIClass* bus, uint8_t chipSelect, uint32_t speed>
#define MCP23S17_OPT MCP23S17<bus, chipSelect, speed>

class MCP23S17_Base {
  public:

  private:

};

MCP23S17_CLASS class MCP23S17 : public MCP23S17_Base {
  public:
    void begin();
    void pinMode(uint8_t pin, uint8_t mode);
    void digitalWrite(uint8_t pin, bool level);
    bool digitalRead(uint8_t pin);
    void toggle(uint8_t pin);
    void invert(uint8_t pin);
    void enableInterrupt(uint8_t pin, uint8_t mode);
    void disableInterrupt(uint8_t pin);
    void info();

  private:
    void initHAEN();
    void initPins();
    void detectChips();
    uint8_t detectedChips = 0;
};

#include "mcp23s17.tpp"
#endif