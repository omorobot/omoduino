#include <Arduino.h>
#include "sonar.h"

double _alpha = 0.85;
double _detection_range = 40.0;

SONAR::SONAR(int pin_trigger, int pin_echo) {
    _pin_trigger = pin_trigger;
    _pin_echo = pin_echo;
    _measure_prev = 0.0;
    _detected = false;
}

double SONAR::measure_cm() {
    digitalWrite(_pin_trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(_pin_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(_pin_trigger, LOW);
    //Reads echo pin
    unsigned long duration = pulseIn(_pin_echo, HIGH);
    // Calcualte distance
    double distance = duration * 0.034/2;
    if(distance == 0 || distance > 400) {
    return -1.0;
    } else {
        distance = distance * _alpha + _measure_prev * (1.0-_alpha);
        _measure_prev = distance;
        return distance;
    }
}

bool SONAR::detected() {
    if(_measure_prev > 0.0) {
        if(_measure_prev < _detection_range) {
            _detected = true;
        }
        else {
            _detected = false;
        }
    }
    return _detected;
}

void SONAR::set_range(double cm) {
    _detection_range = cm;
}