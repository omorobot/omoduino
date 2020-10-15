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

class SONAR
{
public:
   SONAR(int pin_trigger, int pin_echo);
   double  measure_cm(void);
   void    set_detection_range(double cm);
   bool    detected();
   void    set_range(double cm);
private:

   double _measure_prev;
   int _pin_trigger;
   int _pin_echo;
   bool _detected;
};

#endif