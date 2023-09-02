#include "../include/hardware/S90_GPIOs_functions.hpp"

#define GPIO_PIN 5 // Use GPIO 21 (WiringPi numbering)

void servoPulse(const double& angleRad, const auto& delayOn){
    // double angleRad = angleDeg*(3.1415926)/180;
    double angleDeg = angleRad*180/(3.1415926);
    double pwmOn = angleDeg*13 + 300;

    // Set the pin high for pwmOn 
    digitalWrite(GPIO_PIN, HIGH);
    usleep(pwmOn);

    // Set the pin low for the remaining cycle
    digitalWrite(GPIO_PIN, LOW);
    usleep(delayOn);
}