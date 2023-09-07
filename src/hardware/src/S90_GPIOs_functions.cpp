#include "../include/hardware/S90_GPIOs_functions.hpp"

// Function to convert a desired angle in Rad into the real position of the micro-servo S90
void servoPulse(const int GPIO_PIN, const double& angleRad){ 

    // Angle conversion in Deg & correspondive time on which the pulse must be on
    double angleDeg = angleRad*180/(3.1415926);
    double pwmOn = angleDeg*11.11 + 300;

    // Set the pin high for pwmOn 
    digitalWrite(GPIO_PIN, HIGH);
    usleep(pwmOn);

    // Set the pin low for the remaining cycle 
    digitalWrite(GPIO_PIN, LOW);
    usleep(20.0*1000 - pwmOn);
}