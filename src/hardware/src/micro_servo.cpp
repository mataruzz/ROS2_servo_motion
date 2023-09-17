#include <string>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include "../include/hardware/micro_servo.hpp"

class microServo {
    private:
        int GPIO_PIN;
        float pwmOnMs{0.0}; // ms on which the squarewave will stay up in ms
        float PwmPeriodMs{20.0}; // Pwm period in ms
        float DegAngle;

        float degToPWM(float DegAngle){
            return this->radTodeg(DegAngle)*11.11 + 300; 
        }

        float radTodeg(float radAngle){
            return radAngle*180/M_PI;
        }

    public:
        microServo(const int gpio_pin, bool set_gpio) { // OUTPUT = 1, INPUT = 0
            GPIO_PIN = gpio_pin;
        
            if (set_gpio)
            {
                pinMode(gpio_pin, OUTPUT);
            }else
            {
                pinMode(gpio_pin, INPUT);
            }
        
        }

        void goToAngle(float RadAngle){
            this->DegAngle = this->radTodeg(RadAngle);
            this->pwmOnMs = this->DegToPWM(DegAngle);

            digitalWrite(GPIO_PIN, HIGH);
            usleep(pwmOnMs*1000); // in us

            digitalWrite(GPIO_PIN, LOW);
            usleep(this->PwmPeriodMs*1000 - this->pwmOnMs);
        }

        void continuousSpeed(){
            // TO DO: use for loop with different low time to change speed of continuous motion
            }

};