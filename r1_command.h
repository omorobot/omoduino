/**
 * @file r1_command.h 
 * @brief R1 driver library is for driving OMO-R1 robot along with other OMOROBOT's proprietary sensor modules using CAN transceiver such as MCP2515 
 * For more info, please visit www.omorobot.com 
 *  
 * @License
 * Copyright (c) OMOROBOT INC. All rights reserved. Copyright (c) Kyuhyong You. All rights reserved. 
 * This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * */
#ifndef _R1_COMMAND_H_
#define _R1_COMMAND_H_

#include <inttypes.h>

#define CAN_MOTOR_CMD_VW_PL     0x71
#define CAN_MOTOR_CMD_DAC_ANGLE 0x72

#define CAN_MOTOR_CMD_VW        0x81
#define CAN_MOTOR_ODO_REQUEST   0x82
#define CAN_MOTOR_ODO_RESET     0x83

#define CAN_MOTOR_CMD_DIFF_V    0x86
#define CAN_MOTOR_CMD_RPM       0x87
#define CAN_MOTOR_CMD_PWM       0x88

enum R1_vehicleType {
    R1_vtype_default,
    R1_vtype_PL153
};
enum R1_controlModeType{
    ControlMode_vw,
    ControlMode_diffv,
    ControlMode_rpm,
    ControlMode_dac_angle
};
enum PL153_LiftModeType {
    PL153_lift_stop = 0,
    PL153_lift_up   = 1,
    PL153_lift_down = 2
};
typedef struct {
    int16_t V;
    int16_t W;
}CmdVW_type;

typedef struct {
    int16_t V_l_mm_s;
    int16_t V_r_mm_s;
}CmdDiffv_type;

typedef struct {
    int16_t rpm_l;
    int16_t rpm_r;
}CmdRPM_type;

typedef struct {
    int16_t V_dac;
    int16_t angle;
}CmdPL153_V_angle_type;

class MCP2515;

class R1_Command {
public:
    R1_Command();
    void set_vehicle_type(R1_vehicleType type);
    void set_control_mode(R1_controlModeType mode);
    void cmd_VW(int16_t v_mm_s, int16_t w_mrad_s);
    void cmd_diffv(int16_t v_l_mm_s, int16_t v_r_mm_s);
    void odo_request();
    void odo_reset();
private:
    R1_vehicleType          v_type;      //Vehicle type
    struct can_frame        canCmdMsg;
    struct can_frame        canOdoMsg;
    R1_controlModeType      mode;
    CmdVW_type              cmdVW;
    CmdDiffv_type           cmdDiffv;
    CmdPL153_V_angle_type   cmdPL153;
    CmdRPM_type             cmdRPM;
    uint8_t                 aux_byte;
    int can_frame_init(can_frame* frame, int id, int dlc);
};

#endif