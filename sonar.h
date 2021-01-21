/**
 * @file "sonar.h"
 * 
 * @brief Interface for measuring distance using HC-SR04 Ultrasonic Sensor 
 * For more info, please visit www.omorobot.com 
 *  
 * @License
 * Copyright (c) OMOROBOT INC. All rights reserved. Copyright (c) Kyuhyong You. All rights reserved. 
 * This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * 
*/
#ifndef _SONAR_H_
#define _SONAR_H_

#define SONAR_USE_AVERAGE_FILTER
//#define SONAR_USE_COMPLEMENTARY_FILTER
#ifdef SONAR_USE_AVERAGE_FILTER
#define SONAR_FILTER_NUM         10
#endif

class SONAR
{
   typedef enum {
      SONAR_TYPE_TRIGGER_ECHO = 0,        //Sonar type is trigger, echo type
      SONAR_TYPE_ANALOG       = 1         //Sonar type is analog
   }SonarType;
public:
   SONAR(int analogPin);
   SONAR(int pin_trigger, int pin_echo);
   double      measure_cm(void);
   int         measure_analog(void);
   bool        detected();
   void        set_range(int cm);
   void        set_enable(bool);
   //void        set_analogRange(int);
private:
   SonarType   sonarType;
   int         _distance_prev;
#ifdef SONAR_USE_AVERAGE_FILTER
   int         _distance_arr[SONAR_FILTER_NUM];
#endif
   int         distance_cm;
   int         _detection_range;
   uint32_t    _measure_cnt;
   int         _pin_trigger;
   int         _pin_echo;
   int         _pin_analog;
   bool        _enabled;
};

#endif