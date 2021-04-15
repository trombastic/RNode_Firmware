// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license.

// Modifications and additions copyright 2018 by Mark Qvist
// Obviously still under the MIT license.

#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <SPI.h>
#if defined(__AVR_ATmega1284P__)
    #define LORA_DEFAULT_SS_PIN    10
    #define LORA_DEFAULT_RESET_PIN 9
    #define LORA_DEFAULT_DIO0_PIN  2
    #define PA_OUTPUT_RFO_PIN      0
    #define PA_OUTPUT_PA_BOOST_PIN 1
  #elif defined(ESP32)
      #define LORA_DEFAULT_SS_PIN    18
      #define LORA_DEFAULT_RESET_PIN 14
      #define LORA_DEFAULT_DIO0_PIN  26
      #define PA_OUTPUT_RFO_PIN      0
      #define PA_OUTPUT_PA_BOOST_PIN 1
      /*!
       * RegPaDac
       */
      #define RF_PADAC_20DBM_MASK                         0xF8
      #define RF_PADAC_20DBM_ON                           0x07
      #define RF_PADAC_20DBM_OFF                          0x04  // Default
      /*!
       * RegPaConfig
       */
      #define RF_PACONFIG_PASELECT_MASK                   0x7F
      #define RF_PACONFIG_PASELECT_PABOOST                0x80
      #define RF_PACONFIG_PASELECT_RFO                    0x00 // Default
      
      #define RF_PACONFIG_MAX_POWER_MASK                  0x8F
      
      #define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

  #endif

#define RSSI_OFFSET 157

class LoRaClass : public Stream {
public:
  LoRaClass();

  int16_t begin(int32_t frequency);
  void end();

  int16_t beginPacket(int16_t implicitHeader = false);
  int16_t endPacket();

  int16_t parsePacket(int16_t size = 0);
  int16_t packetRssi();
  uint8_t packetRssiRaw();
  uint8_t packetSnrRaw();
  float packetSnr();
  int32_t packetFrequencyError();

  // from Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(int16_t));

  void receive(int16_t size = 0);
  void idle();
  void sleep();
  void setTxPower(int16_t level, int16_t outputPin = PA_OUTPUT_PA_BOOST_PIN);
  uint32_t getFrequency();
  void setFrequency(int32_t frequency);
  void setSpreadingFactor(int16_t sf);
  int32_t getSignalBandwidth();
  void setSignalBandwidth(int32_t sbw);
  void setCodingRate4(int16_t denominator);
  void setPreambleLength(int32_t length);
  void setSyncWord(int16_t sw);
  uint8_t modemStatus();
  void enableCrc();
  void disableCrc();

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  byte random();

  void setPins(int16_t ss = LORA_DEFAULT_SS_PIN, int16_t reset = LORA_DEFAULT_RESET_PIN, int16_t dio0 = LORA_DEFAULT_DIO0_PIN);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  static void onDio0Rise();

  void handleLowDataRate();

private:
  SPISettings _spiSettings;
  int16_t _ss;
  int16_t _reset;
  int16_t _dio0;
  int32_t _frequency;
  int16_t _packetIndex;
  int16_t _implicitHeaderMode;
  void (*_onReceive)(int16_t);
};

extern LoRaClass LoRa;

#endif
