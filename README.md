# Micro-servo S90 hardware interface for RaspberryPi 3B+
<p align="center">
  <img width = "150" src="https://github.com/mataruzz/arduino_components_tests/blob/main/S90_servo_motor/images/micro-servo-motor-sg90.jpg">
</p>
The micro-servo S90 is a small, lightweight motorized device commonly used with Raspberry Pi or Arduino boards to control the precise movement of mechanical parts. It is compact, low-cost, and ideal for applications such as robotics, automation, and remote-controlled systems.
The <em>primary objective</em> of this repository is to develop and incorporate the hardware interface, enabling the utilization within the ROS (Robot Operating System) framework.

&NewLine;

For a better understanding and simple control (outside ROS) of the micro servo, see [S90_servo_motor repository](https://github.com/mataruzz/raspberryPi_components_tests/tree/main/S90_servo_motor).

## Configuration and Setup
To <strong>enable</strong> the utilization of this repository, the micro-servo S90's PWM wire <strong>must be connected</strong> to <strong>PIN 5</strong> (GPIO's pin enumeration, corresponding to physical PIN 29).

<p align="center">
  <img width = "450" src="https://github.com/mataruzz/ROS2_servo_motion/blob/humble/src/description/images/gpio_pinout.png">
</p>

### Used System
This repository has been designed for use with the following hardware and software:
- Raspberry Pi 3B+
- Ubuntu Server 22.04 LST
- ROS2 Humble

### Installation 
Once the hardware connection is ensured, access to the Raspberry (e.g., SSH) and:

* Clone the repository both on your PC (to visualize the simulation) and Raspberry (to send actual signals):
```
git clone https://github.com/mataruzz/ROS2_servo_motion
```
* Build the repository, limitating the number of threads on the Raspberry (simple build on your PC)
```
cd ROS2_servo_motion
colcon build --parallel-workers 2 --executor sequential
```
* Source the ws:
```
source install/setup.bash
```
