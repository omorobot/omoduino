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
#include <mcp2515.h>

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

enum R1_MessageType{
    R1MSG_ODO,
    R1MSG_LINEPOS,
    R1MSG_LINEOUT
};

class MCP2515;
class OMOROBOT_R1;

class R1_CanBus {
public:
    typedef void (OMOROBOT_R1::*R1_NewCanRxEvent)(struct can_frame canRxMsg);
    R1_CanBus(void);
    R1_CanBus(uint16_t cspin);
    R1_CanBus(MCP2515* mcp2515);
    void begin_bus(void);
    void scan(void);
    void onNewCanRx(OMOROBOT_R1* obj, R1_NewCanRxEvent cbEvent);
    void set_vehicle_type(R1_vehicleType type);
    void set_control_mode(R1_controlModeType mode);
    void cmd_VW(int16_t v_mm_s, int16_t w_mrad_s);
    void cmd_diffv(int16_t v_l_mm_s, int16_t v_r_mm_s);
    void request_odo(bool reset);
    void set_pl_lift_mode(PL153_LiftModeType mode);

private:
    MCP2515 *_mcp2515;
    R1_NewCanRxEvent        _cbCanRxEvent;
    OMOROBOT_R1*            _cbObj;
    R1_vehicleType          v_type;      //Vehicle type
    typedef struct CanCommandType{
        uint8_t cmd_byte;
        uint16_t cmd1;
        uint16_t cmd2;
        uint8_t aux_byte;
    }CanCommandType;
    
    
    struct can_frame        _canRxMsg;
    R1_controlModeType      mode;

    CanCommandType          _canCmd;
    struct can_frame        _can_tx_odo;
    struct can_frame        _can_tx_cmd;
    bool _can_extern;
    void sendCommand(CanCommandType cmd);
    int can_TxMsg_init(can_frame* frame, int id, int dlc);
};

#endif