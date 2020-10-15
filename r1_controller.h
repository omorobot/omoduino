/**
 * @file r1_controller.h 
 * @brief R1 driver library is for driving OMO-R1 robot along with other OMOROBOT's proprietary sensor modules using CAN transceiver such as MCP2515 
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

#define PID_LINE_KP_DEFAULT         9.0
#define PID_LINE_KI_DEFAULT         0.1
#define PID_LINE_KD_DEFAULT         0.0
#define PID_LINE_ERROR_I_MAX        250.0
#define PID_LINE_OUT_MAX            180.0
#define PID_LINE_FILTER_ALPHA_DEFAULT 0.8
#define V_CONTROL_ACCEL_DEFAULT         1

typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double error_prev;
    double error_i;
    double error_i_max;
    double out_max;
}PID_Type;

class R1_Controller {
public:
    R1_Controller();
    void    set_target_v(int v);
    int     speed_control(int cmd_v, bool go_flag);
    int     line_control_vw(int linePos);
    int     line_control_angle(int linePos);
    void    set_pid_gain_line(PID_Type);
private:
    PID_Type _pid_l;
    int _v_accel;
    int _v_dir;
    int _w_dir;
    int _v_target;
    double _line_filter_alpha;
};
#endif