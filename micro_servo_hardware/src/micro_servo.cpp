#include "../include/micro_servo_hardware/micro_servo.hpp"

// Implementation of private member functions
float microServo::degToPWM(float& degAngle) {
    return degAngle * 11.11 + 300;  // In [us]
}

float microServo::radTodeg(float& radAngle) {
    return radAngle * 180 / M_PI;
}

// Implementation of public member functions
microServo::microServo(const int& gpio_pin) {
    GPIO_PIN = gpio_pin;
    // Setting as default pin as OUTPUT due to S90 working principle
    pinMode(GPIO_PIN, OUTPUT);
}

void microServo::goToAngle(float& RadAngle) {
    degAngle = radTodeg(RadAngle);
    pwmOnUs = degToPWM(degAngle);

    digitalWrite(GPIO_PIN, HIGH);
    usleep(pwmOnUs);

    digitalWrite(GPIO_PIN, LOW);
    usleep((pwmPeriodMs*1000 - pwmOnUs));
}

void microServo::continuousSpeed() {
    // Implementation of continuousSpeed function
}
