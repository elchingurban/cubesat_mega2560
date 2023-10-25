#include "sensor_data.h"

#define NUM_SERVOS 10
#define SERVO_PINS {3, 4, 6, 7, 5, 8, 9, 10, 11, 12}
#define SERVO_MIN_ANGLES {100, 100, 100, 100, 0, 0, 30, 30, 0, 0}
#define SERVO_MAX_ANGLES {180, 180, 180, 180, 180, 180, 80, 80, 60, 30}

Servo servos[NUM_SERVOS];
int servoPins[NUM_SERVOS] = SERVO_PINS;
int servoMinAngles[NUM_SERVOS] = SERVO_MIN_ANGLES;
int servoMaxAngles[NUM_SERVOS] = SERVO_MAX_ANGLES;

void setup(void) {
  Serial.begin(9600);
  // BTSerial.begin(9600);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
  }
  for (uint8_t i = 1; i <= 10; i++) {
    servos[i].write(90);
  }

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
  AGS_setup();
  GPS_Setup();
  delay(2000);
}

int servoNumber = -1;  // Initialize servoNumber to an invalid value
int angle = 0;        // Initialize angle to a default value

void loop() {
  while (Serial.available()) {
    int servoNumber = Serial.parseInt() - 1; // Subtract 1 because arrays are 0-indexed
    if (Serial.read() == ',') {
      int angle = Serial.parseInt();
      if (servoNumber >= 0 && servoNumber < NUM_SERVOS) {
        angle = constrain(angle, servoMinAngles[servoNumber], servoMaxAngles[servoNumber]);
        servos[servoNumber].write(angle);
      }
    }
  }
  // Check for new angle command
  if (Serial.available()) {
    char inputChar = Serial.read();

    if (inputChar == ',') {
      // Validate servo number
      if (servoNumber >= 0 && servoNumber < NUM_SERVOS) {
        angle = constrain(angle, servoMinAngles[servoNumber], servoMaxAngles[servoNumber]);
        servos[servoNumber].write(angle);
      }
      // Reset servoNumber to an invalid value
      servoNumber = -1;
    } else if (inputChar >= '0' && inputChar <= '9') {
      // Build the servoNumber from consecutive digits
      if (servoNumber == -1) {
        servoNumber = inputChar - '0';
      } else {
        servoNumber = servoNumber * 10 + (inputChar - '0');
      }
    }
  }
  sendSensorData();
  // delay(100);
}