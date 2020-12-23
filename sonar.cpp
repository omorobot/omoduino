#include <Arduino.h>
#include "sonar.h"

double      _alpha = 0.85;       //Complementary filter value
int         _detection_range = 40;
double      _analog_to_cm_gain = 1.0;

SONAR::SONAR(int analogPin)
{
   sonarType = SONAR_TYPE_ANALOG;
   _pin_analog = analogPin;
   _distance_prev = 0;
   _detected = false;
   _enabled = true;
}

SONAR::SONAR(int pin_trigger, int pin_echo) {
   sonarType = SONAR_TYPE_TRIGGER_ECHO;
   _pin_trigger = pin_trigger;
   _pin_echo = pin_echo;
   pinMode(_pin_trigger, OUTPUT);
   pinMode(_pin_echo,  INPUT);
   _distance_prev = 0;
   _detected = false;
   _enabled = true;
}

double SONAR::measure_cm() {
   double distance;
   if(!this->_enabled) {
      _distance_prev = distance = -1.0;
      return distance;
   }
   if(sonarType == SONAR_TYPE_TRIGGER_ECHO) {
      digitalWrite(_pin_trigger, LOW);
      delayMicroseconds(2);
      digitalWrite(_pin_trigger, HIGH);
      delayMicroseconds(10);
      digitalWrite(_pin_trigger, LOW);
      //Reads echo pin
      unsigned long duration = pulseIn(_pin_echo, HIGH);
      // Calcualte distance
      distance = duration * 0.034/2;
      if(distance == 0 || distance > 400) {
      return -1.0;
      } else {
         distance = distance * _alpha + (double)_distance_prev * (1.0-_alpha);
         _distance_prev = (int)distance;
         return distance;
      }
   } else if(sonarType == SONAR_TYPE_ANALOG) {
      int sum = 0;
      for(int i=0; i<9;i++){
         _distance_arr[i] = _distance_arr[i+1];
         sum += _distance_arr[i];
      }
      _distance_arr[9] = analogRead(_pin_analog) * _analog_to_cm_gain;
      sum += _distance_arr[9];
      distance = sum /10.0;
      //distance = (double)analogRead(_pin_analog) * _analog_to_cm_gain;
      //distance = distance * _alpha + (double)_distance_prev*(1.0-_alpha);
      _distance_prev = (int)distance;
      return distance;
   }
}


bool SONAR::detected() {
   if(!_enabled) {
      return false;
   }
   if(_distance_prev > 0.0) {
      if(_distance_prev < _detection_range) {
         _detected = true;
      }
      else {
         _detected = false;
      }
   }
   return _detected;
}
/**
 * @brief set detection threshold value
 * */
void SONAR::set_range(int range) {
   _detection_range = range;
}

void SONAR::set_enable(bool en) {
   _enabled = en;
}