/**
 * @file r1_controller.h 
 * @brief R1 구동제어기의 라인 Following 알고리즘 및 제어 방법을 정의 
 * For more info, please visit www.omorobot.com 
 *  
 * @License
 * Copyright (c) OMOROBOT INC. All rights reserved. Copyright (c) Kyuhyong You. All rights reserved. 
 * This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * */
#ifndef _R1_CONTROLLER_H_
#define _R1_CONTROLLER_H_

#define PID_LINE_KP_DEFAULT            9.0
#define PID_LINE_KI_DEFAULT            0.1
#define PID_LINE_KD_DEFAULT            0.0
#define PID_LINE_ERROR_I_MAX_DEFAULT   250.0
#define PID_LINE_OUT_MAX_DEFAULT       180.0
#define PID_LINE_FILTER_ALPHA_DEFAULT  0.95 //was 0.8
#define V_CONTROL_ACCEL_DEFAULT        1

/// PID 제어 게인 구조체
typedef struct PID_Type{
   double      Kp;            ///< P 게인
   double      Ki;            ///< I 게인
   double      Kd;            ///< D 게인
   double      error_prev;    ///< 이전 에러값
   double      error_i;       ///< 에러 누적값
   double      error_i_max;   ///< 에러 누적 최대값
   double      out_max;       ///< Output 최대값
   uint64_t    last_update_millis;  ///< 마지막 업데이트 millis
}PID_Type;

class R1_Controller {
public:
   R1_Controller();
   void     set_target_v(int v);
   void     set_v_accel(int accel);
   int      speed_w_control(int origin_val, int target_val, int increase);
   int      speed_control(int cmd_v, bool go_flag);
   int      line_control_vw(double linePos);
   int      line_control_angle(double linePos);
   void     set_pid_gain_line(PID_Type);
   void     reset_pid_line(void);
private:
   PID_Type          _pid_l;
   int               _v_accel;
   int               _v_dir;
   int               _w_dir;
   int               _v_target;
   double            _line_filter_alpha;
};
#endif