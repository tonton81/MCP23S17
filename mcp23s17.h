#if !defined(_MCP23S17_H_)
#define _MCP23S17_H_

#include "Arduino.h"
#include "circular_buffer.h"

typedef void (*mcp23s17_class_ptr)();

typedef enum MCP23S17_CHIP {
  MCP23S17_0 = 0,
  MCP23S17_1 = 1,
  MCP23S17_2 = 2,
  MCP23S17_3 = 3,
  MCP23S17_4 = 4,
  MCP23S17_5 = 5,
  MCP23S17_6 = 6,
  MCP23S17_7 = 7
} MCP23S17_CHIP;

typedef enum MCP23S17_BANK {
  BANK_0 = 0,
  BANK_1 = 1
} MCP23S17_BANK;

#define MCP23S17_CLASS template<SPIClass* bus, uint8_t chipSelect, uint32_t speed = 2000000>
#define MCP23S17_FUNC template<SPIClass* bus, uint8_t chipSelect, uint32_t speed>
#define MCP23S17_OPT MCP23S17<bus, chipSelect, speed>

typedef void (*_MCP23S17_pin_ptr)();

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
    void defaults();
    void toggle(uint8_t pin);
    void invert(uint8_t pin, bool yes = 1);
    void writeGPIO(MCP23S17_CHIP chip, uint8_t value, MCP23S17_BANK bank);
    void writeGPIO(MCP23S17_CHIP chip, uint16_t value);
    void disableInterrupt(uint8_t pin);
    void info();
    void events();
    void attachInterrupt(uint8_t pin, _MCP23S17_pin_ptr handler, uint8_t type);
    void setCache(uint16_t micros) { gpioCacheTimeout = micros; }
  private:
    Circular_Buffer<uint8_t, 128> interruptQueue;
    _MCP23S17_pin_ptr _pinHandlers[128] = { nullptr }; 
    void readRegisters();
    void initHAEN();
    void initDefaults();
    void detectChips();
    void enableInterrupt(uint8_t pin, uint8_t mode);
    uint8_t detectedChips = 0;
    uint32_t chipData[8][11] = { { 0 } , { 0 } };
    uint32_t counter_GPIO[8] = { 0 };
    void checkInterrupt(uint8_t addr, uint16_t data);
    uint16_t gpioCacheTimeout = 50;
};

#include "mcp23s17.tpp"
#endif
