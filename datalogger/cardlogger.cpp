#include <Arduino.h>

#ifndef cardlogger_h
#define cardlogger_h
#include "cardlogger.h"
#endif

#include <SD.h>

#define LED_STAT1   5     // PD5

/*
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 or 53 on mega
 
 For mega check
 http://arduino.cc/de/Reference/SPI
 
 */

const int chipSelect = 4;
const String seperator = ";";

String  directory;

CardLogger::CardLogger()
{
}

void CardLogger::init()
{
  while (!Serial)
  {
  }

  pinMode(chipSelect, OUTPUT);

  if (!SD.begin())
  {
    // infinite loop with blinking LED_STAT1
    while(1)
    {
      digitalWrite(LED_STAT1, LOW); delay(100);
      digitalWrite(LED_STAT1, HIGH); delay(100);
    }
    return;
  }

  String directoryPreFix = "log_";

  for (int i = 0; i <= 500; i++)
  {
    String dir  = directoryPreFix;
    dir.concat(i);
    char charDir[dir.length()+1];
    dir.toCharArray(charDir, sizeof(charDir));

    if (!SD.exists(charDir))
    {
      if(SD.mkdir(charDir))
      {
        directory =  String(charDir);
        break;
      } 
    }
  }
}

void CardLogger::logXYAngle(XYAngle angle)
{
  String s(directory);
  s.concat("/angle.csv");
  char charDir[s.length()+1];
  s.toCharArray(charDir, sizeof(charDir));

  File dataFile = SD.open(charDir, FILE_WRITE);

  dataFile.print(millis());
  dataFile.print(seperator);
  dataFile.print(angle.angleX);
  dataFile.print(seperator);
  dataFile.print(angle.angleY);
  dataFile.print(seperator);
  dataFile.println(angle.heading);

  dataFile.close(); 
}

void CardLogger::logIMU(IMUValues values)
{
  String s(directory);
  s.concat("/imu.csv");
  char charDir[s.length()+1];
  s.toCharArray(charDir, sizeof(charDir));

  File dataFile = SD.open(charDir, FILE_WRITE);

  dataFile.print(millis());

  dataFile.print(seperator);
  dataFile.print(values.accX);

  dataFile.print(seperator);
  dataFile.print(values.accY);

  dataFile.print(seperator);
  dataFile.print(values.accZ);

  dataFile.print(seperator);
  dataFile.print(values.gyroX);

  dataFile.print(seperator);
  dataFile.print(values.gyroY);

  dataFile.print(seperator);
  dataFile.println(values.gyroZ);

  dataFile.close();
}

void CardLogger::logRC(RCInput values)
{
  String s(directory);
  s.concat("/rc.csv");
  char charDir[s.length()+1];
  s.toCharArray(charDir, sizeof(charDir));

  File dataFile = SD.open(charDir, FILE_WRITE);


  dataFile.print(millis());

  dataFile.print(seperator);
  dataFile.print(values.roll);

  dataFile.print(seperator);
  dataFile.print(values.pitch);

  dataFile.print(seperator);
  dataFile.print(values.yaw);

  dataFile.print(seperator);
  dataFile.println(values.throttle);

  dataFile.close();
} 

void CardLogger::logGPS(GPSValues values)
{
  String s(directory);
  s.concat("/gps.csv");
  char charDir[s.length()+1];
  s.toCharArray(charDir, sizeof(charDir));

  File dataFile = SD.open(charDir, FILE_WRITE);

  dataFile.print(millis());

  dataFile.print(seperator);
  dataFile.print(values.hasFix);

  dataFile.print(seperator);
  dataFile.print(values.satNumber);

  dataFile.print(seperator);
  dataFile.print(values.lat);

  dataFile.print(seperator);
  dataFile.print(values.lng);

  dataFile.print(seperator);
  dataFile.print(values.altitude);

  dataFile.print(seperator);
  dataFile.println(values.groundSpeed);

  dataFile.close(); 
}

void CardLogger::logGPS_Time(GPS_TimeValues values)
{
  String s(directory);
  s.concat("/gps_time.csv");
  char charDir[s.length()+1];
  s.toCharArray(charDir, sizeof(charDir));

  File dataFile = SD.open(charDir, FILE_WRITE);

  dataFile.print(millis());

  dataFile.print(seperator);
  dataFile.print(values.week);

  dataFile.print(seperator);
  dataFile.println(values.tow);

  dataFile.close(); 
}

