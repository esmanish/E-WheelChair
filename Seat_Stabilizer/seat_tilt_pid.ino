#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define PWM 5
#define DIR 4

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 50;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

double angleX, angleY, angleZ;
uint8_t systemM, gyro, accel, mag = 0;
int8_t boardTemp;

const bool FORWARD = HIGH;
const bool BACKWARD = LOW;
const int limit = 1;
const int pwm = 220;
const int backEnd = -16;
const int frontEnd = 26;

// PID Constants
double Kp = 1;  // Proportional gain
double Ki = 0;  // Integral gain
double Kd = 2;  // Derivative gain

double setpoint = 0.0;  // Desired orientation
double prevError = 0.0;
double integral = 0.0;

void getCaliberation() {
  bno.getCalibration(&systemM, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(systemM);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);
}

double getOrientation(sensors_event_t* event) {
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    angleX = event->orientation.x;
    angleY = event->orientation.y;
    angleZ = event->orientation.z;
  }

  Serial.print("\tx= ");
  Serial.print(angleX);
  Serial.print(" |\ty= ");
  Serial.print(angleY);
  Serial.print(" |\tz= ");
  Serial.println(angleZ);

  return angleZ;
}

void getTemperature() {
  boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);
}

void motorRun(bool dir, int spd) {
  Serial.print(dir);
  Serial.print("   ");
  Serial.println(spd);

  digitalWrite(DIR, dir);
  analogWrite(PWM, spd);
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  delay(1000);
}

void loop(void) {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double currentOrientation = getOrientation(&orientationData);

  // Calculate the error
  double error = setpoint - currentOrientation;

  // PID control calculations
  double output = Kp * error + Ki * integral + Kd * (error - prevError);

  // Update integral and previous error
  integral += error;
  prevError = error;

  // Map the output to the PWM range
  int pwmValue = map(output, -180, 180, 0, 255);
  Serial.println(pwmValue);
  // Set the motor speed
  //  motorRun(FORWARD, pwmValue);

  getCaliberation();

  if (gyro != 3) {
    Serial.println("Reset");
    if (!bno.begin()) {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
    }
  }

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  if (getOrientation(&orientationData)  < -limit ) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    Serial.println("Moving Forward");
    motorRun(FORWARD, pwm);

  } else if (getOrientation(&orientationData)  > limit) {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    Serial.println("Moving Backward");
    motorRun(BACKWARD, pwm);

  } else {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    Serial.println("Stopped");
    motorRun(FORWARD, 0);
    //delay(2000);

  }

  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | Output: ");
  Serial.print(output);
  Serial.print(" | PWM: ");
  Serial.println(pwmValue);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
