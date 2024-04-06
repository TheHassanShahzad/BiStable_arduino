Self-Balancing Robot Project with ESP32 and microROS

This project utilizes an ESP32 microcontroller to control a two-wheeled self-balancing robot. The main functionality includes:

microROS Subscriber Node: The main file runs on an ESP32 microcontroller and starts a microROS subscriber node. This node listens to commands, including inclination targets and steering commands, from a ROS 2 node. It also obtains increments/decrements to PID gains.

Inclination Estimation with MPU6050: Data from an MPU6050 IMU sensor is read and filtered to estimate the inclination of the robot. This information is crucial for maintaining balance.

PID Controller for Control Signal: A PID controller is implemented to obtain control signals necessary to reach the desired inclination targets. The PID controller helps the robot adjust its position in real-time to maintain balance.

Stepper Motor Control: The control signal generated by the PID controller and the steering commands are combined to drive stepper motors, which in turn drive the robot's wheels. This allows for precise control of the robot's movement and balance.
