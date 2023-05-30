
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>

#include <Adafruit_Sensor.h>

#define SEALEVELPRESSURE_HPA (1013.25)

#define BMP280_ADDRESS 0x76
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_CTRL_MEAS 0xF4
#define BMP280_CONFIG 0xF5
#define BMP280_RESET 0xE0

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BMP280 bmp; // use I2C interface

#define NUM_SERVOS 10
#define SERVO_PINS {3, 4, 6, 7, 5, 8, 9, 10, 11, 12}
#define SERVO_MIN_ANGLES {100, 100, 100, 100, 0, 0, 30, 30, 0, 0}
#define SERVO_MAX_ANGLES {180, 180, 180, 180, 180, 180, 80, 80, 60, 30}

Servo servos[NUM_SERVOS];
int servoPins[NUM_SERVOS] = SERVO_PINS;
int servoMinAngles[NUM_SERVOS] = SERVO_MIN_ANGLES;
int servoMaxAngles[NUM_SERVOS] = SERVO_MAX_ANGLES;

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
} struct_message;
struct_message myData;
unsigned long delayTime;

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

void setup(void) {
  Serial.begin(115200);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
  }
  servos[1].write(90);
  servos[2].write(90);
  servos[3].write(90);
  servos[4].write(90);
  servos[5].write(90);
  servos[6].write(90);
  servos[7].write(90);
  servos[8].write(90);
  servos[9].write(90);
  servos[10].write(90);

  Wire.begin();
  MPU_setup();
  //HMC5883 configuration
  if(!mag.begin())
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  /* Display some basic information on this sensor */
  // displaySensorDetails();
  BMP_setup();
  delay(2000);
}

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

  // DynamicJsonDocument doc(1024);
  // doc["acceleration"]["x"] = myData.Xacc;
  // doc["acceleration"]["y"] = myData.Yacc;
  // doc["acceleration"]["z"] = myData.Zacc;
  // doc["gyroscope"]["x"] = myData.AngaccX;
  // doc["gyroscope"]["y"] = myData.AngaccY;
  // doc["gyroscope"]["z"] = myData.AngaccZ;
  // Serialize the JSON object and send it
  String output;
  // serializeJsonPretty(doc, Serial);
  output = String(myData.Xacc) + "," + String(myData.Yacc) +"," + String(myData.Zacc) + "," + String(myData.AngaccX)+ "," + 
           String(myData.AngaccY)+ "," + String(myData.AngaccZ);

  Serial.println(output);
}

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

  // Format data into a single string
  String output = String(myData.Xacc) + "," + String(myData.Yacc) + "," + String(myData.Zacc) + "," +
                  String(myData.AngaccX) + "," + String(myData.AngaccY) + "," + String(myData.AngaccZ) + "," +
                  String(myData.MagX) + "," + String(myData.MagY) + "," + String(myData.MagZ) + "," +
                  String(myData.Pressure) + "," + String(myData.Temperature) + "," + String(myData.Altitude);

  Serial.println(output);
}

void loop() {
  
  if (Serial.available()) {
    int servoNumber = Serial.parseInt() - 1; // Subtract 1 because arrays are 0-indexed
    if (Serial.read() == ',') {
      int angle = Serial.parseInt();
      if (servoNumber >= 0 && servoNumber < NUM_SERVOS) {
        angle = constrain(angle, servoMinAngles[servoNumber], servoMaxAngles[servoNumber]);
        servos[servoNumber].write(angle);
      }
    }
  }
  sendSensorData();
  // MPUloop();
  // HMCloop();
  // BMPloop();
  delay(2000);
}