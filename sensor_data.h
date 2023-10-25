
#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>

#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>

#include "AGS02MA.h"
#include <ACS712.h>

#include <SoftwareSerial.h>
#include <TinyGPS.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define BMP280_ADDRESS 0x76

TinyGPS gps;
SoftwareSerial ss(17, 16);
SoftwareSerial BTSerial(18,19);

#define batteryIn A14     // pin where the OUT pin from sensor is connected on Arduino
#define panelIn A15       // pin where the OUT pin from sensor is connected on Arduino

int mVperAmp = 185;       // this the 5A version of the ACS712 -use 100 for 20A Module and 66 for 30A Module
int Watt = 0;
double Voltage = 0;
double VRMS = 0;
float AmpsRMS = 0;

unsigned long delayTime;

AGS02MA AGS(26);
Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP280 bmp; // use I2C interface

//struct for RF Data
typedef struct struct_message {
  float Xacc;
  float Yacc;
  float Zacc;
  float AngaccX;
  float AngaccY;
  float AngaccZ;
  float MagX;
  float MagY;
  float MagZ;
  float Temperature;
  float Pressure;
  float Altitude;
  float Heading;
  float PPMValue;
  float Watt;
  float Voltage;
  float AmpsRMS;
  float Latitude;
  float Longtitude;
} struct_message;

struct_message myData;

void AGS_setup(void);
void BMP_setup(void);
void MPU_setup(void);
void BMPloop(void);
void MPUloop(void);
void HMCloop(void);
void sendSensorData();

//AGS setup
void AGS_setup(void)
{
  bool b = AGS.begin();
  b = AGS.setPPBMode();
  uint8_t m = AGS.getMode();
}

//BMP setup
void BMP_setup(void)
{
  Wire.begin();
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS);
}

//MPU setup
void MPU_setup(void)
{
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

float getVPP()
{
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 1024;             // store max value here
  int minValue = 0;          // store min value here
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
     readValue = analogRead(batteryIn);
   }
   
  // Subtract min from max
  result = ((readValue * 5.0) / 1024.0);      
  return result;
}

void GPS_Setup(void)
{
  ss.begin(4800);
}

void sendDataToProMini(String& data) {
  Wire.beginTransmission(8);  // Replace '8' with the I2C address of the Pro Mini
  Wire.write(data.c_str());  // Send the data as a char array
  Wire.endTransmission();
}

void sendSensorData() {
  // Collect sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sensors_event_t event; 
  mag.getEvent(&event);

  Voltage = getVPP();
  VRMS = (Voltage/2.0) * 0.707;   //root 2 is 0.707
  AmpsRMS = (VRMS * 1000) / mVperAmp;
  Watt = VRMS * AmpsRMS;    // 1.3 is an empirical calibration factor

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
  }

  myData.Xacc = a.acceleration.x;
  myData.Yacc = a.acceleration.y;
  myData.Zacc = a.acceleration.z;
  myData.AngaccX = g.gyro.x;
  myData.AngaccY = g.gyro.y;
  myData.AngaccZ = g.gyro.z;
  myData.MagX = event.magnetic.x;
  myData.MagY = event.magnetic.y;
  myData.MagZ = event.magnetic.z;
  myData.Pressure = bmp.readPressure();
  myData.Temperature = bmp.readTemperature();
  myData.Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  myData.PPMValue = AGS.readPPB();
  myData.AmpsRMS = AmpsRMS;
  myData.Watt = Watt;
  myData.Voltage = Voltage;

  // Format data into a single string
  String output = String(myData.Xacc) + "," + String(myData.Yacc) + "," + String(myData.Zacc) + "," +
                  String(myData.AngaccX) + "," + String(myData.AngaccY) + "," + String(myData.AngaccZ) + "," +
                  String(myData.MagX) + "," + String(myData.MagY) + "," + String(myData.MagZ) + "," +
                  String(myData.Pressure) + "," + String(myData.Temperature) + "," + String(myData.Altitude) + 
                  String(myData.PPMValue) + "," + String(myData.AmpsRMS) + "," + String(myData.Watt) + "," + String(myData.Voltage);
  // TX_buffer[0] = output.c_str();
  Serial.println(output);
  if (Serial.available()) {
    BTSerial.println(output);
  }  
  int dataSize = output.length();
  int chunks = dataSize / 32 + 1; // Split data into chunks of 32 bytes

  for (int i = 0; i < chunks; i++) {
    int start = i * 32;
    int end = min(dataSize, start + 32);
  Wire.beginTransmission(8); // Pro Mini's I2C address
  Wire.write(output.c_str());
  Wire.endTransmission();
  delay(100); // Delay to avoid overwhelming the receiver
  // }
}
#endif
