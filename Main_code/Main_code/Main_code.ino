#include <PID_v1.h>
#include "MPU6050Aries.h"
#include <Wire.h>
#include <math.h>

// L298N Motor Driver Pin Definitions
#define ENA 6   // Left motor PWM
#define IN1 7   // Left motor direction
#define IN2 8   // Left motor direction
#define ENB 5   // Right motor PWM
#define IN3 4   // Right motor direction
#define IN4 3   // Right motor direction

// PID parameters
double Kp = 40;
double Ki = 40;
double Kd = 0.05;
double targetAngle = 0;  // Desired pitch angle in degrees
double input, output;       // PID input (current pitch) and output (motor power)
PID pid(&input, &output, &targetAngle, Kp, Ki, Kd, DIRECT);

// Create MPU6050Aries object (our custom library using Kalman filter)
MPU6050Aries mpu;

// Function to control motors via L298N
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -leftSpeed);
  }
  
  // Right Motor
  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, rightSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -rightSpeed);
  }
}

void setup() {
  // Initialize motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize I2C and Serial communication
  Wire.begin();
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing MPU6050Aries...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected! Check wiring.");
    while (1);
  }
  Serial.println("MPU6050Aries initialized.");
  
  // Initialize the PID controller
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  pid.SetSampleTime(5);  // Sample time in ms
  
  Serial.println("Self-balancing robot system initialized.");
}

void loop() {
  // Update sensor data (this updates the Kalman-filtered pitch)
  mpu.update();
  
  // Use the filtered pitch as the PID input
  input = mpu.getPitch(); // Pitch value from MPU6050Aries library
  
  // Compute the PID output based on the error (input - targetAngle)
  pid.Compute();
  
  // Constrain the motor power output
  int motorPower = constrain((int)output, -255, 255);
  
  // Drive both motors with the computed power
  setMotors(motorPower, motorPower);
  
  // Debug output: display pitch and PID output
  Serial.print("Pitch: ");
  Serial.print(input, 2);
  Serial.print(" | PID Output: ");
  Serial.println(motorPower);
  
  delay(5);  // Adjust delay for control loop timing as needed
}
