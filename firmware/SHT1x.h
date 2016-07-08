/**
 * SHT1x Library
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 * Based on previous work by:
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */
#ifndef SHT1x_h
#define SHT1x_h

#include "application.h"

#define TIMEOUT_MILLIS 1000

enum {
  SHT1X_CMD_MEASURE_TEMP  = 0x03,
  SHT1X_CMD_MEASURE_RH    = 0x05,
  SHT1X_CMD_READ_STATUS   = 0x07,
  SHT1X_CMD_SOFT_RESET    = 0x1E
};

class SHT1x
{
  public:
    SHT1x(int dataPin, int clockPin);
    SHT1x(int dataPin, int clockPin, float voltage, bool intPullup=false);
    void reset();
    //composite functions
    float readHumidity();
    float readTemperatureC();
    float readTemperatureF();
    uint8_t readStatus();
    //decoupled functions
    void requestTemperature();
    int readInTemperature();
    void requestHumidity();
    float readInHumidity();
    float parseHumidity(int raw);
    float parseTemperatureC(int raw);
    float parseTemperatureF(int raw);
  private:
    int _temperatureRaw;
    int _dataPin;
    int _clockPin;
    PinMode _dataInputMode;
    uint8_t _status;
    int _numBits;
    float _D1C; float _D1F; float _D2C; float _D2F;
    float _linearInterpolation(float coeffA, float coeffB, float valB, float input);
    void _setConversionCoeffs(float voltage);

    void sendCommandSHT(uint8_t _command);
    void waitForResultSHT();
    int getDataSHT(int bits);
    void skipCrcSHT();
    bool checkCrcSHT(uint8_t cmd, uint16_t data, int datalen);
    uint8_t crc8(uint8_t data, uint8_t startval);
};

#endif