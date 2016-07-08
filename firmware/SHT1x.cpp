/**
 * SHT1x Library
 *
 * Recent elaborations by Louis Thiery (me@louisthiery.com) <hoping to see a GNU or CC license>
 *
 * Copyright 2009 Jonathan Oxer <jon@oxer.com.au> / <www.practicalarduino.com>
 * Based on previous work by:
 *    Maurice Ribble: <www.glacialwanderer.com/hobbyrobotics/?p=5>
 *    Wayne ?: <ragingreality.blogspot.com/2008/01/ardunio-and-sht15.html>
 *
 * Manages communication with SHT1x series (SHT10, SHT11, SHT15)
 * temperature / humidity sensors from Sensirion (www.sensirion.com).
 */

#include "SHT1x.h"
#include "application.h"

#define DEFAULT_VOLTAGE 5

#define NAN 999999

SHT1x::SHT1x(int dataPin, int clockPin)
{
  _dataPin = dataPin;
  _clockPin = clockPin;
  _dataInputMode = INPUT;
  _setConversionCoeffs(DEFAULT_VOLTAGE);
  _status = 0;
}

SHT1x::SHT1x(int dataPin, int clockPin, float voltage, bool intPullup)
{
  _dataPin = dataPin;
  _clockPin = clockPin;
  _dataInputMode = (intPullup ? INPUT_PULLUP : INPUT);
  _setConversionCoeffs(voltage);
  _status = 0;
}

#define VALS_IN_TABLE 5

// dynamically determines conversion coefficients based on voltage
// tables generated by datasheet
void SHT1x::_setConversionCoeffs(float voltage){
  _D2C =   0.01; // for 14 Bit DEGC
  _D2F =   0.018; // for 14 Bit DEGC
  
  const float coeffsC[] = {40.1,39.8,39.7,39.6,39.4};
  const float coeffsF[] = {40.2,39.6,39.5,39.3,38.9};
  const float vals[] = {5,4,3.5,3,2.5};
  
  for (int i=1; i<VALS_IN_TABLE; i++) {
    if (voltage > vals[i]) {
        _D1C = -_linearInterpolation(coeffsC[i-1], coeffsC[i], vals[i-1], voltage);
        _D1F = -_linearInterpolation(coeffsF[i-1], coeffsF[i], vals[i-1], voltage);
        break;
    }
  }
}

float SHT1x::_linearInterpolation(float coeffA, float coeffB, float valA, float input) {
  Serial.println((coeffA-coeffB)/valA*input+coeffB);
  return (coeffA-coeffB)/valA*input+coeffB;
}


/* ================  Public methods ================ */

/**
 * Resets the sensor, e.g. after communication desync
 */
void SHT1x::reset()
{
  pinMode(_clockPin, OUTPUT);
  pinMode(_dataPin, OUTPUT);
  delay(11);
  for (int i=0; i<9; i++) {
    digitalWrite(_clockPin, HIGH);
    digitalWrite(_clockPin, LOW);
  }
  sendCommandSHT(SHT1X_CMD_SOFT_RESET);
  delay(11);
  pinMode(_dataPin, _dataInputMode);
}

/**
 * Requests, reads, and parses temperatures in C
 */
float SHT1x::readTemperatureC()
{
  //just putting together functions
  requestTemperature();
  int raw = readInTemperature();
  return (raw >= 0) ? parseTemperatureC(raw) : NAN;
}

float SHT1x::parseTemperatureC(int raw){
  return (raw * _D2C) + _D1C;
}


/**
 * Requests, reads, and parses temperatures in F
 */
float SHT1x::readTemperatureF()
{
  requestTemperature();
  int raw = readInTemperature();
  return (raw >= 0) ? parseTemperatureF(raw) : NAN;
}

float SHT1x::parseTemperatureF(int raw){
  // Convert raw value to degrees Fahrenheit
  return (raw * _D2F) + _D1F;
}

/**
 * Temperature Request
 */
void SHT1x::requestTemperature()
{
  sendCommandSHT(SHT1X_CMD_MEASURE_TEMP);
}

/**
 * Read-in Temperature
 */
int SHT1x::readInTemperature()
{
  waitForResultSHT();
  uint16_t rawTemp = getDataSHT(16);
  if (checkCrcSHT(SHT1X_CMD_MEASURE_TEMP, rawTemp, 2)) {
    _temperatureRaw = rawTemp;  // store in variable for humidity
    return _temperatureRaw;
  }
  else {
    return -1;
  }
}

/**
 * Reads current temperature-corrected relative humidity
 */
float SHT1x::readHumidity()
{
  requestHumidity();
  float raw = readInHumidity();
  return (raw >= 0) ? parseHumidity(raw) : NAN;
}

void SHT1x::requestHumidity(){
  // Fetch the value from the sensor
  sendCommandSHT(SHT1X_CMD_MEASURE_RH);
}

float SHT1x::readInHumidity(){
  int val;                    // Raw humidity value returned from sensor
  waitForResultSHT();
  val = getDataSHT(16);
  if (checkCrcSHT(SHT1X_CMD_MEASURE_RH, val, 2)) {
    return val;
  }
  else {
    return -1;
  }
}

float SHT1x::parseHumidity(int raw){
  float linearHumidity;       // Humidity with linear correction applied
  float correctedHumidity;    // Temperature-corrected humidity
  float temperature;          // Raw temperature value

  // Conversion coefficients from SHT15 datasheet
  const float CO1 = -4.0;       // for 12 Bit
  const float CO2 =  0.0405;    // for 12 Bit
  const float CO3 = -0.0000028; // for 12 Bit
  const float TO1 =  0.01;      // for 14 Bit
  const float TO2 =  0.00008;   // for 14 Bit

  // Apply linear conversion to raw value
  linearHumidity = CO1 + CO2 * raw + CO3 * raw * raw;

  //temperature request should be accomplished right before this!
  temperature = parseTemperatureC(_temperatureRaw);

  // Correct humidity value for current temperature
  correctedHumidity = (temperature - 25.0 ) * (TO1 + TO2 * raw) + linearHumidity;

  return correctedHumidity;
}

uint8_t SHT1x::readStatus() {
  uint8_t status;
  sendCommandSHT(SHT1X_CMD_READ_STATUS);
  status = getDataSHT(8);
  if (checkCrcSHT(SHT1X_CMD_READ_STATUS, status, 1)) {
    _status = status; // Store for use in CRC calculations
    return _status;
  }
  else {
    return 0xFF;
  }
}

/**
 */
void SHT1x::sendCommandSHT(uint8_t _command)
{
  // Transmission Start
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, LOW);

  // The command (3 msb are address and must be 000, and last 5 bits are command)
  shiftOut(_dataPin, _clockPin, MSBFIRST, _command);

  // Wait for ACK
  bool ackerror = false;
  pinMode(_dataPin, _dataInputMode);
  digitalWrite(_clockPin, HIGH);
  if (digitalRead(_dataPin) != LOW) ackerror = true;
  digitalWrite(_clockPin, LOW);

  if (_command == SHT1X_CMD_MEASURE_TEMP || _command == SHT1X_CMD_MEASURE_RH) {
    delayMicroseconds(1); /* Give the sensor time to release the data line */
    if (digitalRead(_dataPin) != HIGH) ackerror = true;
  }

  if (ackerror) {
    Serial.println("SHT1x: Sensor did not ACK command");
  }
}

/**
 */
void SHT1x::waitForResultSHT()
{
  pinMode(_dataPin, _dataInputMode);
 
  unsigned long int start = millis();
  
  // Wait for SHT to show that it's ready or move along if timeout is reached
  while( digitalRead(_dataPin) && (millis()-start)<TIMEOUT_MILLIS );
}

/**
 */
int SHT1x::getDataSHT(const int bits=16)
{
  int val = 0;

  pinMode(_clockPin, OUTPUT);
  digitalWrite(_clockPin, LOW);

  if (bits == 16) {
    // Get the most significant bits
    pinMode(_dataPin, _dataInputMode);
    val = shiftIn(_dataPin, _clockPin, MSBFIRST);
    val *= 256;

    // Send the required ack
    pinMode(_dataPin, OUTPUT);

    //digitalWrite(_dataPin, HIGH);
    digitalWrite(_dataPin, LOW);
    digitalWrite(_clockPin, HIGH);
    digitalWrite(_clockPin, LOW);
  }

  // Get the least significant bits
  pinMode(_dataPin, _dataInputMode);
  val |= shiftIn(_dataPin, _clockPin, MSBFIRST);

  return val;
}

/**
 */
void SHT1x::skipCrcSHT()
{
  // Skip acknowledge to end trans (no CRC)
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);

  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);
}

/**
 */
bool SHT1x::checkCrcSHT(const uint8_t cmd, const uint16_t data, const int datalen)
{
  // Pull data line low to ACK
  pinMode(_dataPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);

  digitalWrite(_dataPin, LOW);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);

  pinMode(_dataPin, _dataInputMode);
  int crcVal = shiftIn(_dataPin, _clockPin, MSBFIRST);

  pinMode(_dataPin, OUTPUT);
  digitalWrite(_dataPin, HIGH);
  digitalWrite(_clockPin, HIGH);
  digitalWrite(_clockPin, LOW);
  pinMode(_dataPin, _dataInputMode);

  // Calculate CRC from status register, command and data
  uint8_t crcCalc;
  crcCalc = crc8(cmd, _status & 0xF);
  if (datalen == 2) crcCalc = crc8((data >> 8) & 0xFF, crcCalc);
  crcCalc = crc8(data & 0xFF, crcCalc);
  return (crcVal == crcCalc);
}

uint8_t SHT1x::crc8(const uint8_t data, const uint8_t startval)
{
  /* Algorithm based on the Sensirion CRC8 calculation app note
   * Note! variable crc is reversed compared to the app note
   */
  uint8_t crc = startval;

  for (int i = 7; i >= 0; i--) {
    crc = (((data >> i) & 1) == (crc & 1)) ? crc >> 1 : ((crc >> 1) ^ 0xC) | 0x80;
  }
  return crc;
}