#ifndef MICRO_SERVO_HPP
#define MICRO_SERVO_HPP

#include "wiringPi.h"
#include <unistd.h>
#include <cstdio>



class microServo {
    private:
        int GPIO_PIN;
        float pwmOnMs{0.0}; // ms on which the squarewave will stay up in ms
        float PwmPeriodMs{20.0}; // Pwm period in ms
        float DegAngle;

        float degToPWM(float DegAngle);

        float radTodeg(float radAngle);

    public:
        microServo(const int gpio_pin, bool set_gpio);

        void goToAngle(float RadAngle);

        void continuousSpeed();

};