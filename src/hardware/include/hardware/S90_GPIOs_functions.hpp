#ifndef S90_GPIOS_FUNCTIONS_HPP
#define S90_GPIOS_FUNCTIONS_HPP

// Define the GPIO_PIN
// #define GPIO_PIN 5

// #include "~/projects/ROS2_servo_motion/src/WiringPi/wiringPi/wiringPi.h"
#include "wiringPi.h"
#include <unistd.h>
#include <cstdio>

// Function declaration
void servoPulse(const int GPIO_PIN, const double& angleDeg);

#endif // S90_GPIOS_FUNCTIONS_HPP
