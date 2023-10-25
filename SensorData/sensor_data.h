#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include "AGS02MA.h"

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
} struct_message;

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

//BMP loop
void BMPloop(void)
{
  myData.Pressure = bmp.readPressure();
  myData.Temperature = bmp.readTemperature();
  myData.Altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);  
  // can now print out the new measurements
  Serial.print(F("Temperature = "));
  Serial.print(myData.Temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(myData.Pressure);
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(myData.Altitude); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();
  delay(2000);
}

//MPU loop
void MPUloop(void)
{
/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  myData.Xacc = a.acceleration.x;
  myData.Yacc = a.acceleration.y;
  myData.Zacc = a.acceleration.z;

  myData.AngaccX = g.gyro.x;
  myData.AngaccY = g.gyro.y;
  myData.AngaccZ = g.gyro.z;

  String output;
  output = String(myData.Xacc) + "," + String(myData.Yacc) +"," + String(myData.Zacc) + "," + String(myData.AngaccX)+ "," + 
           String(myData.AngaccY)+ "," + String(myData.AngaccZ);

  Serial.println(output);
}

//HMC loop
void HMCloop(void)
{
  //HMC5883 works..........................................................................
  sensors_event_t event; 
  mag.getEvent(&event);

  myData.MagX = event.magnetic.x;
  myData.MagY = event.magnetic.y;
  myData.MagZ = event.magnetic.z;
  
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // Rabat is close to 0 degree.
  float declinationAngle = 0;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
    
  // Convert radians to degrees for readability.
  myData.Heading = heading * 180/M_PI;
  String output;
  output += String(myData.MagX) + "," + String(myData.MagY) +"," + String(myData.MagZ);

  Serial.println(output);
}

void sendSensorData() {
  // Collect sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sensors_event_t event; 
  mag.getEvent(&event);

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

  // Format data into a single string
  String output = String(myData.Xacc) + "," + String(myData.Yacc) + "," + String(myData.Zacc) + "," +
                  String(myData.AngaccX) + "," + String(myData.AngaccY) + "," + String(myData.AngaccZ) + "," +
                  String(myData.MagX) + "," + String(myData.MagY) + "," + String(myData.MagZ) + "," +
                  String(myData.Pressure) + "," + String(myData.Temperature) + "," + String(myData.Altitude) + String(myData.PPMValue);
  ELECHOUSE_cc1101.SendData(output.c_str(), output.length());
  Serial.println(output);
}

float getVPP()
{
  float result;
  int readValue;                // value read from the sensor
  int maxValue = 0;             // store max value here
  int minValue = 1024;          // store min value here
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(panelIn);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the minimum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;
      
   return result;
 }

#endif
