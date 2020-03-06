#include <mcp23s17.h>
#include "Arduino.h"

MCP23S17_FUNC void MCP23S17_OPT::begin() {
  ::pinMode(chipSelect, OUTPUT);
  ::digitalWriteFast(chipSelect, HIGH);
  initHAEN(); /* enable hardware addressing */
  detectChips(); /* detect chips */
  initPins(); /* initialize pin configurations */
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

MCP23S17_FUNC void MCP23S17_OPT::initPins() {
  for ( uint8_t addr = 0; addr < 8; addr++ ) {
    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (addr << 1)) << 8) | 0x0);
    bus->transfer16(0xFFFF); /* all inputs */
    bus->transfer16(0x0000); /* non-inverted logic */
    bus->transfer16(0x0000); /* disables gpio interrupt-on-change */
    bus->transfer16(0x0000); /* default compare register for interrupt-on-change */
    bus->transfer16(0x0000); /* pin value is compared against the previous pin value */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0));
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (addr << 1)) << 8) | 0xC);
    bus->transfer16(0x0000); /* disable all pullups */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();
  }
}

MCP23S17_FUNC void MCP23S17_OPT::disableInterrupt(uint8_t pin) {
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
    uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin); /* disable interrupt */
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write GPINTEN register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x4);
    bus->transfer16(__builtin_bswap16(data));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read INTCON register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x8);
    data = __builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin);
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write INTCON register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x8);
    bus->transfer16(__builtin_bswap16(data));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read DEFVAL register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x6);
    data = __builtin_bswap16(bus->transfer16(0xFFFF)) & ~(1UL << pin);
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write DEFVAL register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x6);
    bus->transfer16(__builtin_bswap16(data));
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

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read INTCON register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x8);
    uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    ( mode == CHANGE ) ? data &= ~(1UL << pin) : data |= (1UL << pin);

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write INTCON register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x8);
    bus->transfer16(__builtin_bswap16(data));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read DEFVAL register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x6);
    data = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    ( mode == RISING ) ? data &= ~(1UL << pin) : data |= (1UL << pin);

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write DEFVAL register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x6);
    bus->transfer16(__builtin_bswap16(data));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read GPINTEN register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x4);
    data = __builtin_bswap16(bus->transfer16(0xFFFF)) | (1UL << pin);
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write GPINTEN register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x4);
    bus->transfer16(__builtin_bswap16(data));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
}

MCP23S17_FUNC void MCP23S17_OPT::invert(uint8_t pin) {
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
    uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    data = (data & ~(1UL << pin)) | (bool)!(data & (1UL << pin)) << pin;

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write IPOL register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x2);
    bus->transfer16(__builtin_bswap16(data));
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

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x12);
    uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    data = (data & ~(1UL << pin)) | (bool)!(data & (1UL << pin)) << pin;

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write port register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x12);
    bus->transfer16(__builtin_bswap16(data));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
}

MCP23S17_FUNC bool MCP23S17_OPT::digitalRead(uint8_t pin) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return 0;
  uint16_t data = 0;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x12);
    data = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();
  
    break;
  }
  return (data & (1UL << pin));
}

MCP23S17_FUNC void MCP23S17_OPT::digitalWrite(uint8_t pin, bool level) {
  if ( pin >= (__builtin_popcount(detectedChips) * 16U) ) return;
  for ( uint8_t i = 0; i < 8; i++ ) {
    if ( !(detectedChips & (1U << i)) ) continue;
    if ( pin > 15 ) {
      pin -= 16;
      continue;
    }

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read port register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0x12);
    uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    data = (data & ~(1UL << pin)) | (level << pin); /* set new pin state */

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write port register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x12);
    bus->transfer16(__builtin_bswap16(data));
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
    uint16_t data = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    ( mode == OUTPUT ) ? data &= ~(1UL << pin) : data |= (1UL << pin); /* output(0) or input(1)? */

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write direction register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0x0);
    bus->transfer16(__builtin_bswap16(data));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* read pullup register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x41 | (i << 1)) << 8) | 0xC);
    data = __builtin_bswap16(bus->transfer16(0xFFFF));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    ( mode == INPUT_PULLUP ) ? data |= (1UL << pin) : data &= ~(1UL << pin); /* pullup(1)? */

    bus->beginTransaction(SPISettings(speed,MSBFIRST,SPI_MODE0)); /* write pullup register */
    ::digitalWriteFast(chipSelect, LOW);
    bus->transfer16(((0x40 | (i << 1)) << 8) | 0xC);
    bus->transfer16(__builtin_bswap16(data));
    ::digitalWriteFast(chipSelect, HIGH);
    bus->endTransaction();

    break;
  }
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
        Serial.printf("(0b%u%u%u%u%u%u%u%u%u%u%u%u%u%u%u%u)", (bool)(data & (1UL << 15)), (bool)(data & (1UL << 14)), (bool)(data & (1UL << 13)), (bool)(data & (1UL << 12)), (bool)(data & (1UL << 11)), (bool)(data & (1UL << 10)), (bool)(data & (1UL << 9)), (bool)(data & (1UL << 8)), (bool)(data & (1UL << 7)), (bool)(data & (1UL << 6)), (bool)(data & (1UL << 5)), (bool)(data & (1UL << 4)), (bool)(data & (1UL << 3)), (bool)(data & (1UL << 2)), (bool)(data & (1UL << 1)), (bool)(data & (1UL << 0)) );
        Serial.print("  ");
      }
      Serial.println();
      ::digitalWriteFast(chipSelect, HIGH);
      bus->endTransaction();
  }



  Serial.println("\n\n");

}





