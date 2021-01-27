#include <Arduino.h>
#include "sonar.h"



#ifdef USE_COMPLEMENTARY_FILTER
double      _alpha = 0.85;       //Complementary filter value
#endif
double      _analog_to_cm_gain = 1.0;

SONAR::SONAR(int analogPin)
{
   sonarType = SONAR_TYPE_ANALOG;
   _pin_analog = analogPin;
   _distance_prev = 0;
   _enabled = true;
   this->_measure_cnt = 0;
   this->_detection_range = 40;
}

SONAR::SONAR(int pin_trigger, int pin_echo) {
   sonarType = SONAR_TYPE_TRIGGER_ECHO;
   _pin_trigger = pin_trigger;
   _pin_echo = pin_echo;
   pinMode(_pin_trigger, OUTPUT);
   pinMode(_pin_echo,  INPUT);
   _distance_prev = 0;
   _enabled = true;
}

double SONAR::measure_cm() {
   double distance;
   if(!this->_enabled) {
      this->distance_cm = distance = -1.0;
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
         this->_measure_cnt++;
         distance = (double)analogRead(_pin_analog) * _analog_to_cm_gain;
#ifdef SONAR_USE_COMPLEMENTARY_FILTER
         distance = distance * _alpha + (double)_distance_prev * (1.0-_alpha);
#endif
         _distance_prev = (int)distance;
         this->distance_cm = distance;
         return distance;
      }
   } else if(sonarType == SONAR_TYPE_ANALOG) {
      this->_measure_cnt++;
      distance = (double)analogRead(_pin_analog) * _analog_to_cm_gain;
#ifdef SONAR_USE_AVERAGE_FILTER
      int sum = 0;
      for(int i=0; i<(SONAR_FILTER_NUM-1);i++){
         _distance_arr[i] = _distance_arr[i+1];
         sum += _distance_arr[i];
      }
      _distance_arr[SONAR_FILTER_NUM-1] = distance;
      sum += _distance_arr[SONAR_FILTER_NUM-1];
      distance = sum /SONAR_FILTER_NUM;
#elif defined USE_COMPLEMENTARY_FILTER
      distance = distance * _alpha + (double)_distance_prev*(1.0-_alpha);
#endif
      this->distance_cm = (int)distance;
      return distance;
   }
}


bool SONAR::detected() {
   bool detected = false;
   if(!_enabled) {
      return false;
   }
#ifdef SONAR_USE_AVERAGE_FILTER
   if(this->_measure_cnt < SONAR_FILTER_NUM+5) {
      return false;
   }
#endif
   if(this->distance_cm > 0.0) {
      if(this->distance_cm < this->_detection_range) {
         detected = true;
      }
      else {
         detected = false;
      }
   }
   return detected;
}
/**
 * @brief set detection threshold value
 * */
void SONAR::set_range(int range) {
   this->_detection_range = range;
}

void SONAR::set_enable(bool en) {
   _enabled = en;
}