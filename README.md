This repository contains the flight software and control logic for a 2-axis PID Thrust Vector Control (TVC) system designed for amateur rocketry. 

HARDWARE REQUIREMENTS:

Microcontroller: ESP32 WROOM32D

IMU: DFROBOT FERMION 10DOF [ITG3200 (Gyro) & ADXL345 (Accel)].

Actuators: 2x servos for the gimbal mechanism.

Interface: 1x Activation switch, 2x Status LEDs (OK/ERR).



LIBRARY DEPENDENCIES: 

Ensure you have ESP32Servo, Wire, and the specific drivers for ITG3200 and ADXL345 installed in your Arduino IDE.

CALIBRATION: 

Update the gyro_bias_dps and accel_offset arrays in the code with your specific sensor calibration data.





CONTROL LOOP:

The system utilizes an ESP32 microcontroller to process IMU data through a complementary filter, calculating Euler angles. 
Then, since the IMU may be mounted with an offset, the software performs a real-time coordinate transformation using a rotation matrix to align sensor data with the rocket's physical axes, 
besides applying fixed offsets to ensure the sensor reads 0º when the rocket is standing in it's vertical position. Later, the real Euler angles are fed to the PID controller, which follows the next control law 
$$u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de_{filt}(t)}{dt}$$ (featuring a filtered derivative term and an integral anti-windup).
Finally, the output of the PID is sent to the servos, whose job is to steer the rocket via the gimbaling of the rocket motor.
