#include <mcp23s17.h>
#include "Arduino.h"

MCP23S17_FUNC void MCP23S17_OPT::begin() {
  ::pinMode(chipSelect, OUTPUT);
  ::digitalWriteFast(chipSelect, HIGH);

  for ( uint8_t addr = 0; addr < 8; addr++ ) { /* check if any chips are already initialized */
    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (addr << 1)) << 8) | 0xA);
    if ( __builtin_bswap16(bus->transfer16(0xFFFF)) == 0x1818 ) {
       ::digitalWriteFast(chipSelect, HIGH);
       bus->endTransaction();
       detectChips(); /* detect chips */
       readRegisters(); /* copy registers to memory */
       return;
    }
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();
  }
  /* no chips initialized, configuring defaults */
  defaults();
}

MCP23S17_FUNC void MCP23S17_OPT::defaults() {
  initHAEN(); /* enable hardware addressing */
  detectChips(); /* detect chips */
  initDefaults(); /* initialize default configuration */
  readRegisters(); /* copy registers to memory */
}

MCP23S17_FUNC void MCP23S17_OPT::initHAEN() {
  bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
  ::digitalWriteFast(chipSelect, LOW);
  bus->transfer16(0x400A);
  bus->transfer(0x18);
  ::digitalWriteFast(chipSelect, HIGH);
  bus->endTransaction();

  bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
  ::digitalWriteFast(chipSelect, LOW);
  bus->transfer16(0x4E0A);
  bus->transfer(0x18);
  ::digitalWriteFast(chipSelect, HIGH);
  bus->endTransaction();
}

MCP23S17_FUNC void MCP23S17_OPT::initDefaults() {
  for ( uint8_t addr = 0; addr < 8; addr++ ) {
    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (addr << 1)) << 8) | 0x0);
    bus->transfer16(0xFFFF); /* IODIR */
    bus->transfer16(0x0000); /* IPOL */
    bus->transfer16(0x0000); /* GPINTEN */
    bus->transfer16(0x0000); /* DEFVAL */
    bus->transfer16(0x0000); /* INTCON */
    bus->transfer16(0x1818); /* IOCON */
    bus->transfer16(0x0000); /* GPPU */
    bus->transfer16(0x0000); /* INTF */
    bus->transfer16(0x0000); /* INTCAP */
    bus->transfer16(0x0000); /* GPIO */
    bus->transfer16(0x0000); /* LATCH */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();
    counter_GPIO[addr] = micros();
  }
}

MCP23S17_FUNC void MCP23S17_OPT::detachInterrupt(uint8_t pin) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read registers */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x4);
    chipData[i][2] = __builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin); /* GPINTEN */
    chipData[i][3] = __builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin); /* DEFVAL */
    chipData[i][4] = __builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin); /* INTCON */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write registers */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x4);
    bus->transfer16(__builtin_bswap16(chipData[i][2])); /* GPINTEN */
    bus->transfer16(__builtin_bswap16(chipData[i][3])); /* DEFVAL */
    bus->transfer16(__builtin_bswap16(chipData[i][4])); /* INTCON */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
}

MCP23S17_FUNC void MCP23S17_OPT::enableInterrupt(uint8_t pin, uint8_t mode) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read GPINTEN register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x4);
    chipData[i][2] = __builtin_bswap16(bus->transfer16(0xFFFF)) | (1UL << pin); /* GPINTEN */
    chipData[i][3] = (__builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin)) | (( mode == RISING ) ? 0 : (1UL << pin) );
    chipData[i][4] = (__builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin)) | (( mode == CHANGE ) ? 0 : (1UL << pin) );
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write GPINTEN register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x4);
    bus->transfer16(__builtin_bswap16(chipData[i][2])); /* GPINTEN */
    bus->transfer16(__builtin_bswap16(chipData[i][3])); /* DEFVAL */
    bus->transfer16(__builtin_bswap16(chipData[i][4])); /* INTCON */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
}

MCP23S17_FUNC void MCP23S17_OPT::invert(uint8_t pin, bool yes) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read IPOL register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x2);
    chipData[i][1] = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    chipData[i][1] = (chipData[i][1] & ~(1UL << pin)) | ((yes) ? (1UL << pin) : 0);

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write IPOL register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x2);
    bus->transfer16(__builtin_bswap16(chipData[i][1]));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
}

MCP23S17_FUNC void MCP23S17_OPT::toggle(uint8_t pin) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    if (micros() - counter_GPIO[i] > gpioCacheTimeout) {
      bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
      ::digitalWriteFast(chipSelect, LOW);
      bus->transfer16(((0x41 | (i << 1)) << 8) | 0x12);
      uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF)); /* Read GPIOs */
      ::digitalWriteFast(chipSelect, HIGH);
      bus->endTransaction();
      checkInterrupt(i, data);
    }

    chipData[i][9] = (chipData[i][9] & ~(1UL << pin)) | (bool)!(chipData[i][9] & (1UL << pin)) << pin;

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write port register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x12);
    bus->transfer16(__builtin_bswap16(chipData[i][9]));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
}

MCP23S17_FUNC bool MCP23S17_OPT::digitalRead(uint8_t pin) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return 0;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    if (micros() - counter_GPIO[i] < gpioCacheTimeout) return ( chipData[i][9] & (1UL << pin) );

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x12);
    uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF)); /* Read GPIOs */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();
    checkInterrupt(i, data);
    return (chipData[i][9] & (1UL << pin));
    break;
  }
  return 0;
}

MCP23S17_FUNC void MCP23S17_OPT::digitalWrite(uint8_t pin, bool level) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    if (micros() - counter_GPIO[i] > gpioCacheTimeout) {
      bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
      ::digitalWriteFast(chipSelect, LOW);
      bus->transfer16(((0x41 | (i << 1)) << 8) | 0x12);
      uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF)); /* Read GPIOs */
      ::digitalWriteFast(chipSelect, HIGH);
      bus->endTransaction();
      checkInterrupt(i, data);
    }

    chipData[i][9] = (chipData[i][9] & ~(1UL << pin)) | (level << pin);

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write port register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x12);
    bus->transfer16(__builtin_bswap16(chipData[i][9]));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
}

MCP23S17_FUNC void MCP23S17_OPT::pinMode(uint8_t pin, uint8_t mode) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read direction register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x0);
    chipData[i][6] = (__builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin)) | (( mode == OUTPUT ) ? 0 : (1UL << pin) );
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write direction register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x0);
    bus->transfer16(__builtin_bswap16(chipData[i][6]));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read pullup register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0xC);
    chipData[i][6] = (__builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin)) | (( mode == INPUT_PULLUP ) ? (1UL << pin) : 0 );
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write pullup register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0xC);
    bus->transfer16(__builtin_bswap16(chipData[i][6]));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
}

MCP23S17_FUNC void MCP23S17_OPT::writeGPIO(MCP23S17_CHIP chip, uint8_t value, MCP23S17_BANK bank) {
  bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write port register */
  ::digitalWriteFast(chipSelect, LOW);
  bus->transfer16(((0x40 | (chip << 1)) << 8) | 0x12 );
  chipData[chip][9] = (!bank) ? ((chipData[chip][9] & 0xFF00) | value) : ((chipData[chip][9] & 0x00FF) | (value << 8));
  bus->transfer16(__builtin_bswap16(chipData[chip][9]));
  ::digitalWriteFast(chipSelect, HIGH);
  bus->endTransaction();
  counter_GPIO[chip] = micros();
}

MCP23S17_FUNC void MCP23S17_OPT::writeGPIO(MCP23S17_CHIP chip, uint16_t value) {
  bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write port register */
  ::digitalWriteFast(chipSelect, LOW);
  bus->transfer16(((0x40 | (chip << 1)) << 8) | 0x12);
  bus->transfer16(__builtin_bswap16(value));
  chipData[chip][9] = value; 
  ::digitalWriteFast(chipSelect, HIGH);
  bus->endTransaction();
  counter_GPIO[chip] = micros();
}

MCP23S17_FUNC void MCP23S17_OPT::readRegisters() {
 for ( uint8_t addr = 0; addr < 8; addr++ ) {
    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (addr << 1)) << 8) | 0x0);
    for ( uint8_t i = 0; i < 11; i++ ) {
      chipData[addr][i] = __builtin_bswap16(bus->transfer16(0xFFFF)); /* Dump chip registers */
    }
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();
    counter_GPIO[addr] = micros();
  }
}

MCP23S17_FUNC void MCP23S17_OPT::attachInterrupt(uint8_t pin, _MCP23S17_pin_ptr handler, uint8_t type) {
  _pinHandlers[pin] = handler;
  if ( type == CHANGE ) enableInterrupt(pin, CHANGE);
  if ( type == RISING ) enableInterrupt(pin, RISING);
  if ( type == FALLING ) enableInterrupt(pin, FALLING);
}

MCP23S17_FUNC void MCP23S17_OPT::events() {
  for ( uint8_t addr = 0; addr < 8; addr++ ) {
    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (addr << 1)) << 8) | 0x12);
    uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF)); /* Dump GPIO */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();
    checkInterrupt(addr, data);
  }

  if ( interruptQueue.size() ) {
    uint8_t pin = interruptQueue.read();
    if ( _pinHandlers[pin] ) _pinHandlers[pin]();
  }
}

MCP23S17_FUNC void MCP23S17_OPT::checkInterrupt(uint8_t addr, uint16_t data) {
  if ( chipData[addr][9] != data ) {
    for ( uint8_t i = 0; i < 16; i++ ) {
      if ( !(chipData[addr][2] & (1UL << i)) ) continue; /* GPINTEN: skip non interrupt enabled pins */
      if ( !(chipData[addr][4] & (1UL << i)) ) { /* INTCON: ON-CHANGE INTERRUPT */
        if ( (chipData[addr][9] & (1UL << i)) != (data & (1UL << i)) ) {
          bool found = 0;
          for ( uint8_t c = 0; c < interruptQueue.size(); c++ ) {
            if ( interruptQueue.peek(c) == ((addr*16)+i) ) {
              found = 1;
              break;
            }
          }
          if ( !found ) interruptQueue.write((addr*16)+i);
        }
      }
      else {
        if ( !(chipData[addr][3] & (1UL << i)) ) { /* DEFVAL: ON RISING */
          if ( !(chipData[addr][9] & (1UL << i)) && (data & (1UL << i)) ) {
            bool found = 0;
            for ( uint8_t c = 0; c < interruptQueue.size(); c++ ) {
              if ( interruptQueue.peek(c) == ((addr*16)+i) ) {
                found = 1;
                break;
              }
            }
            if ( !found ) interruptQueue.write((addr*16)+i);
          } 
        }
        else { /* DEFVAL: ON FALLING */
          if ( (chipData[addr][9] & (1UL << i)) && !(data & (1UL << i)) ) {
            bool found = 0;
            for ( uint8_t c = 0; c < interruptQueue.size(); c++ ) {
              if ( interruptQueue.peek(c) == ((addr*16)+i) ) {
                found = 1;
                break;
              }
            }
            if ( !found ) interruptQueue.write((addr*16)+i);
          } 
        }
      }
    }
    chipData[addr][9] = data; /* Update GPIO register */
  }
  counter_GPIO[addr] = micros();
}

MCP23S17_FUNC void MCP23S17_OPT::detectChips() {
  for ( uint8_t addr = 0; addr < 8; addr++ ) {
    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (addr << 1)) << 8) | 0xA);
    if ( __builtin_bswap16(bus->transfer16(0xFFFF)) == 0x1818 ) detectedChips |= (1U << addr);
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();
  }
}

MCP23S17_FUNC void MCP23S17_OPT::info() {
  if ( !detectedChips ) {
    Serial.println("\n\nNo MCP23S17's detected!");
    return;
  }
  Serial.printf("\n\nDetected chips:\n------------------------------------------------------------------------------------\n");
  Serial.printf("GPIO:  15   14   13   12   11   10    9    8    7    6    5    4    3    2    1    0\n");
  Serial.printf("------------------------------------------------------------------------------------\n");
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
      bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
      ::digitalWriteFast(chipSelect, LOW);
      bus->transfer16(((0x41 | (i << 1)) << 8) | 0x12);
      uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF));
      checkInterrupt(i, data);
      ::digitalWriteFast(chipSelect, HIGH);
      bus->endTransaction();
      Serial.printf("Addr%u:\t%u    %u    %u    %u    %u    %u    %u    %u    %u    %u    %u    %u    %u    %u    %u    %u\n",i, (bool)(data & (1UL << 15)), (bool)(data & (1UL << 14)), (bool)(data & (1UL << 13)), (bool)(data & (1UL << 12)), (bool)(data & (1UL << 11)), (bool)(data & (1UL << 10)), (bool)(data & (1UL << 9)), (bool)(data & (1UL << 8)), (bool)(data & (1UL << 7)), (bool)(data & (1UL << 6)), (bool)(data & (1UL << 5)), (bool)(data & (1UL << 4)), (bool)(data & (1UL << 3)), (bool)(data & (1UL << 2)), (bool)(data & (1UL << 1)), (bool)(data & (1UL << 0)) );
  }

  Serial.printf("\n\nRegisters, HEX format:\n--------------------------------------------------------------------------------------------------------------------\n");
  Serial.printf("         IODIR      IPOL     GPINTEN   DEFVAL    INTCON    IOCON      GPPU      INTF     INTCAP     GPIO      OLAT\n");
  Serial.printf("--------------------------------------------------------------------------------------------------------------------\n");

  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;

      bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
      ::digitalWriteFast(chipSelect, LOW);
      bus->transfer16(((0x41 | (i << 1)) << 8) | 0x0);

      Serial.printf("Addr%u:\t",i);

      for ( uint16_t p = 0, data = 0; p < 11; p++ ) {
        data = __builtin_bswap16(bus->transfer16(0xFFFF));
        if ( p == 9 ) checkInterrupt(i, data);
        Serial.printf("(0x%04X)", data);
        Serial.print("  ");
      }
      Serial.println();
      ::digitalWriteFast(chipSelect, HIGH);
      bus->endTransaction();
  }

  Serial.printf("\n\nRegisters, BIN format:\n--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
  Serial.printf("                IODIR                 IPOL                 GPINTEN               DEFVAL                INTCON                IOCON                 GPPU                   INTF                INTCAP                 GPIO                  OLAT\n");
  Serial.printf("--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");

  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;

      bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
      ::digitalWriteFast(chipSelect, LOW);
      bus->transfer16(((0x41 | (i << 1)) << 8) | 0x0);

      Serial.printf("Addr%u:\t",i);

      for ( uint16_t p = 0, data = 0; p < 11; p++ ) {
        data = __builtin_bswap16(bus->transfer16(0xFFFF));
        if ( p == 9 ) checkInterrupt(i, data);
        Serial.printf("(0b%u%u%u%u%u%u%u%u%u%u%u%u%u%u%u%u)", (bool)(data & (1UL << 15)), (bool)(data & (1UL << 14)), (bool)(data & (1UL << 13)), (bool)(data & (1UL << 12)), (bool)(data & (1UL << 11)), (bool)(data & (1UL << 10)), (bool)(data & (1UL << 9)), (bool)(data & (1UL << 8)), (bool)(data & (1UL << 7)), (bool)(data & (1UL << 6)), (bool)(data & (1UL << 5)), (bool)(data & (1UL << 4)), (bool)(data & (1UL << 3)), (bool)(data & (1UL << 2)), (bool)(data & (1UL << 1)), (bool)(data & (1UL << 0)) );
        Serial.print("  ");
      }
      Serial.println();
      ::digitalWriteFast(chipSelect, HIGH);
      bus->endTransaction();
  }
  Serial.println("\n\n");
}
