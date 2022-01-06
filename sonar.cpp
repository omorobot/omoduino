#include <Arduino.h>
#include "sonar.h"

#ifdef SONAR_USE_COMPLEMENTARY_FILTER
double      _alpha = 0.85;       //Complementary filter value
#endif
//int         _detection_range = 40;
double      _analog_to_cm_gain = 1.0;
/**
 * @brief 아날로그 타입의 SONAR::SONAR 객체를 생성합니다.\n
 * Construct a new SONAR::SONAR object as analog type sensor
 * 
 * @param analogPin analog pin
 */
SONAR::SONAR(int analogPin)
{
   sonarType = SONAR_TYPE_ANALOG;
   _pin_analog = analogPin;
   _distance_prev = 0;
   //_detected = false;
   _enabled = true;
   this->_measure_cnt = 0;
}
/**
 * @brief 트리거&에코 타입의 SONAR::SONAR 객체를 생성합니다.\n
 * Construct a new SONAR::SONAR object as trigger echo type sensor
 * @param pin_trigger Trigger pin
 * @param pin_echo Echo pin
 */
SONAR::SONAR(int pin_trigger, int pin_echo) {
   sonarType = SONAR_TYPE_TRIGGER_ECHO;
   _pin_trigger = pin_trigger;
   _pin_echo = pin_echo;
   pinMode(_pin_trigger, OUTPUT);
   pinMode(_pin_echo,  INPUT);
   _distance_prev = 0;
   //_detected = false;
   _enabled = true;
}
/**
 * @brief 초음파 센서로부터 거리를 측정하고 거리값(cm)을 반환합니다.\n
 * Measure distance in CM from sonar
 * 
 * @return double distance in cm
 */
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
         this->_measure_cnt++;
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
#elif defined SONAR_USE_COMPLEMENTARY_FILTER
      distance = distance * _alpha + (double)_distance_prev*(1.0-_alpha);
      _distance_prev = (int)distance;
#endif
      this->distance_cm = (int)distance;
      return distance;
   }
}
/**
 * @brief 가장 최신의 센서 값을 측정합니다.
 * Get the latest sensor reading
 * @return double distance in cm
 */
double SONAR::get_distance()
{
#ifdef SONAR_USE_AVERAGE_FILTER
   if(_measure_cnt > SONAR_FILTER_NUM) {
      return this->distance_cm;
   } else {
      return -1.0;
   }
#else 
      return this->disatnce_cm;
#endif
}

/**
 * @brief 초음파로 감지된 물체가 장애물 인지거리 이내인지 확인합니다.\n
 * Check if measured distance from object below minimum range
 * 
 * @return true if object within detection range
 * @return false if object beyond detction range
 */
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
 * @brief 초음파 감지 거리를 설정합니다.\n
 * Set detection threshold value.
 * */
void SONAR::set_range(int range) {
   this->_detection_range = range;
}
/**
 * @brief 초음파 센서를 Enable 또는 Disable 합니다.\n
 * Enable/Disable sonar.
 * @param en 
 */
void SONAR::set_enable(bool en) {
   _enabled = en;
}