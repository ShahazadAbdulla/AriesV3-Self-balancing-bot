# Self-Balancing Robot

This project implements a self-balancing robot using an MPU6050 sensor and a PID controller. The robot's balancing and movement are controlled via an L298N motor driver, and the tilt angle is monitored using a custom made Kalman filter-based MPU6050 library. The robot is powered by an **Aries V3.0 RISC-V board**.

## Overview

This robot uses an **MPU6050 sensor** (with a Kalman filter) to detect the angle of tilt, and a **PID controller** is employed to keep the robot balanced. The **L298N motor driver** controls the motors to balance the system.

### Key Features

- **MPU6050 Sensor**: Measures the robot's tilt (pitch) angle and uses a Kalman filter to provide accurate data.
- **PID Controller**: Used to control the motors to maintain balance by adjusting motor speeds based on the tilt.
- **L298N Motor Driver**: Controls the motors' speed and direction to balance the robot.
  
## Hardware Components

1. **Aries V3.0 RISC-V Board**
   - Used to run the custom Kalman filter library and process the sensor data.
  
2. **MPU6050**:
   - 3-axis accelerometer and gyroscope used to measure the robot's tilt.
   - Custom library built for Aries V3.0 board using a Kalman filter to fuse accelerometer and gyroscope data.

3. **L298N Motor Driver**:
   - Dual H-Bridge motor driver used to control two DC motors.

4. **DC Motors**:
   - Used for propulsion and balancing the robot.

6. **Frame**:
   - The 3D printable frame.

### Pinout Connections

- **MPU6050 I2C**:
  - **SDA** to **SDA1** pin (on Aries V3.0)
  - **SCL** to **SCL1** pin (on Aries V3.0)
  - **VCC** to 5V
  - **GND** to Ground
  
- **Motor Driver (L298N)**:
  - **ENA** to **PWM Pin 6**
  - **IN1** to **Pin 7**
  - **IN2** to **Pin 8**
  - **ENB** to **PWM Pin 5**
  - **IN3** to **Pin 4**
  - **IN4** to **Pin 3**
  
## Software Overview

The code uses the following libraries:

- **MPU6050Aries**: Custom library built by ShahazadAbdulla to work with the Aries RISC-V board. It uses a **Kalman filter** to fuse accelerometer and gyroscope data for accurate pitch angle readings.
- **PID_v1**: A popular PID library to implement the control loop.
- **Wire**: Standard Arduino library for I2C communication.

### Setup

1. Clone or download the repository.
2. Install the necessary libraries:
   - `MPU6050Aries` (for Kalman filter-based MPU6050 readings)

   - `PID_v1` (for implementing PID control)
3. Connect the components as per the pinout provided above.
4. Upload the code to your **Aries V3.0 RISC-V board**.
5. Run the self-balancing robot.

### Code Explanation

- **Sensor Data**: The **MPU6050** sensor is used to detect the robot's tilt (pitch angle). The data is processed using the custom Kalman filter-based library to obtain accurate and stable readings.
- **PID Control**: The tilt angle from the sensor is fed into the **PID controller**. Based on the error between the target angle and the actual tilt, the PID controller adjusts the motor speeds to correct any imbalance.

### Frame STL

The STL file for the frame is provided in the 3D print folder. You can 3D print the frame using the file and assemble it with the other components.

---

## Team Members

- **Christy Joys**
- **Alan Joseph**
- **Tharun Oommen Jacob**

## Inspiration

This project is inspired by the following tutorials and resources:

- [FlyRobo - Self-Balancing Robot](https://www.flyrobo.in/blog/self-balancing-robot)
- [Instructables - Arduino Self-Balancing Robot](https://www.instructables.com/Arduino-Self-Balancing-Robot-1/)


## License

This project is open-source and released under the MIT License. Feel free to contribute and modify the project as needed.

---
