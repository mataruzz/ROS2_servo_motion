#include "../include/hardware/S90_GPIOs_functions.hpp"

void servoPulse(const int GPIO_PIN, const double& angleRad){
    // double angleRad = angleDeg*(3.1415926)/180;
    double angleDeg = angleRad*180/(3.1415926);
    double pwmOn = angleDeg*11.11 + 300;

    // Set the pin high for pwmOn 
    digitalWrite(GPIO_PIN, HIGH);
    usleep(pwmOn);

    // Set the pin low for the remaining cycle
    digitalWrite(GPIO_PIN, LOW);
    usleep(20.0*1000 - pwmOn);
}