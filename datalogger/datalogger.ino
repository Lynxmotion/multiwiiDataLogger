#include <SD.h>

#include "protocol.h"

#ifndef serialstate_h
#define serialstate_h
#include "serialstate.h"
#endif

#ifndef cardlogger_h
#define cardlogger_h
#include "cardlogger.h"
#endif

#define INBUF_SIZE 64

#define LED_STAT1   5     // PD5

static uint8_t inBuf[INBUF_SIZE];
static uint8_t checksum;
static uint8_t commandMW;
static uint8_t offset;
static uint8_t dataSize;

static Protocol p;
static CardLogger logger;
int8_t c,n;

static uint16_t GPS_rawTime_week;
static uint32_t GPS_rawTime_tow;

void setup()  
{
  // Set PD5
  pinMode(LED_STAT1, OUTPUT);
  digitalWrite(LED_STAT1, HIGH);
  delay(1000);
  digitalWrite(LED_STAT1, LOW); delay(250);
  digitalWrite(LED_STAT1, HIGH); delay(250);
  digitalWrite(LED_STAT1, LOW); delay(250);
  digitalWrite(LED_STAT1, HIGH); delay(250);
  digitalWrite(LED_STAT1, LOW); delay(250);
  digitalWrite(LED_STAT1, HIGH); delay(250);
  delay(1000);

  Serial.begin(115200);
  logger.init();
}

uint8_t k = 0;

void loop() // run over and over
{
  uint8_t  datad = 0;
  uint8_t  *data = & datad;

  // If you dont need a certain log just comment out the two lines for sending and reading
  p.send_msp(MSP_ATTITUDE, data, 0);
  readData();
  p.send_msp(MSP_RAW_IMU, data, 0);
  readData();
  p.send_msp(MSP_RC, data, 0);
  readData();
  p.send_msp(MSP_RAW_GPS, data, 0);
  readData();
  p.send_msp(MSP_TIME_GPS, data, 0);
  readData();

  // Toggle LED every other pass
  if((++k % 2))
  {
      digitalWrite(LED_STAT1, LOW);
  }
  else
  {
      digitalWrite(LED_STAT1, HIGH);
  }
}

void readData()
{
  delay(60);

  while (Serial.available())
  {
    byte c = Serial.read();

    if (c_state == IDLE)
    {
      c_state = (c=='$') ? HEADER_START : IDLE;
    } 
    else if (c_state == HEADER_START)
    {
      c_state = (c=='M') ? HEADER_M : IDLE;
    } 
    else if (c_state == HEADER_M)
    {
      c_state = (c=='>') ? HEADER_ARROW : IDLE;
    } 
    else if (c_state == HEADER_ARROW)
    {
      if (c > INBUF_SIZE)
      {
        // now we are expecting the payload size
        c_state = IDLE;
      } 
      else
      {
        dataSize = c;
        offset = 0;
        checksum = 0;
        checksum ^= c;
        c_state = HEADER_SIZE;  
      }
    } 
    else if (c_state == HEADER_SIZE)
    {
      commandMW = c;
      checksum ^= c;
      c_state = HEADER_CMD;
    } 
    else if (c_state == HEADER_CMD && offset < dataSize)
    {
      checksum ^= c;
      inBuf[offset++] = c;
    } 
    else if (c_state == HEADER_CMD && offset >= dataSize)
    {
      if (checksum == c)
      {
        if (commandMW == MSP_ATTITUDE)
        {
          XYAngle result = p.evalAtt(inBuf);
          logger.logXYAngle(result);
        }
        if (commandMW == MSP_RAW_IMU)
        {
          IMUValues result = p.evalIMU(inBuf);
          logger.logIMU(result);
        }
        if (commandMW == MSP_RC)
        {
          RCInput result = p.evalRC(inBuf);
          logger.logRC(result);
        }
        if (commandMW == MSP_RAW_GPS)
        {
          GPSValues result = p.evalGPS(inBuf);
          logger.logGPS(result);
        }
        if (commandMW == MSP_RAW_GPS)
        {
          GPS_TimeValues result = p.evalGPS_Time(inBuf);
          logger.logGPS_Time(result);

          // Update global time stamp
          GPS_rawTime_week = result.week;
          GPS_rawTime_tow = result.tow;
        }
      } 
      c_state = IDLE;
    }
  }
}

