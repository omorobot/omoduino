/**
 * @file "line_detector.h"
 * 
 * @brief Interface for line sensor module
 * For more info, please visit www.omorobot.com 
 *  
 * @License
 * Copyright (c) OMOROBOT INC. All rights reserved. Copyright (c) Kyuhyong You. All rights reserved. 
 * This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * 
*/
#ifndef _SENSOR_LINE_H_
#define _SENSOR_LINE_H_

#include <inttypes.h>

#define DEFAULT_LINE_DETECTOR_TIMEOUT_MS        5000

typedef enum {
   LINE_OK = 0,
   LINE_OUT = 1,
   LINE_OUT_TIMEOUT = 2
}LINE_DETECT_RESULT;


class LINE_DETECTOR {
public:
   LINE_DETECTOR();
   LINE_DETECT_RESULT   detect_linePos_from_u16data(uint16_t);
   double      get_line_pos(void);
   bool        is_line_out(void);
   void        set_lineout_timeout_ms(int);
private:
   double      _line_pos;
   double      _line_pos_last;
   bool        _is_line_out;
   int         _lineOut_timer;
   int         _lineOut_timeOut_ms;
   uint64_t    _lineDetect_millis_last;
};

#endif