/**
 * @file r1_command.h 
 * @brief R1 구동 드라이버 제어를 위한 CAN 버스 명령어에 대한 정의
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

#define CAN_MOTOR_CMD_VW_PL      0x71
#define CAN_MOTOR_CMD_DAC_ANGLE  0x72
#define CAN_MOTOR_CMD_TURN       0x7A

#define CAN_MOTOR_CMD_VW         0x81
#define CAN_MOTOR_ODO_REQUEST    0x82
#define CAN_MOTOR_ODO_RESET      0x83

#define CAN_MOTOR_CMD_DIFF_V     0x86
#define CAN_MOTOR_CMD_RPM        0x87
#define CAN_MOTOR_CMD_PWM        0x88

enum R1_VEHICLE_TYPE {
   /// R1 형 모터 드라이버
   VEHICLE_TYPE_R1,
   /// PL153형 모터 드라이버
   VEHICLE_TYPE_PL153
};
enum R1_CONTROL_MODE_TYPE{
   /// V mm/s, mrad/s 제어방식 구동
   CONTROL_MODE_VW,
   /// 좌/우 바퀴 속도 mm/s 제어방식 구동
   CONTROL_MODE_DIFFV,
   /// 좌/우 바퀴 RPM 제어방식 구동
   CONTROL_MODE_RPM,
   /// 전/후진 속도 및 조향각 제어방식 구동
   CONTROL_MODE_DAC_ANGLE
};
enum PL_LIFT_MODE_TYPE {
   PL_LIFT_STOP = 0,
   PL_LIFT_UP   = 1,
   PL_LIFT_DOWN = 2
};

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
   void set_vehicle_type(R1_VEHICLE_TYPE type);
   void set_control_mode(R1_CONTROL_MODE_TYPE mode);
   void cmd_VW(int16_t v_mm_s, int16_t w_mrad_s);
   void cmd_diffv(int16_t v_l_mm_s, int16_t v_r_mm_s);
   void cmd_pl_dac_angle(int16_t dac, int16_t angle);
   void request_odo(bool reset);
   void set_pl_lift_mode(PL_LIFT_MODE_TYPE mode);

private:
   typedef struct CanCommandType{
      /// 제어명령
      uint8_t     cmd_byte;   
      /// 제어 데이터 1
      uint16_t    data_1;  
      /// 제어 데이터 2     
      uint16_t    data_2;
      uint8_t     aux_byte;
   }CanCommandType;

   MCP2515*                _mcp2515;
   R1_NewCanRxEvent        _cbCanRxEvent;
   /// Ojbect to hold OMOROBOT_R1 class
   OMOROBOT_R1*            _cbObj;             
   /// Vehicle type
   R1_VEHICLE_TYPE         _vehicle_type;      

   struct can_frame        _canRxMsg;
   R1_CONTROL_MODE_TYPE    _control_mode;
   CanCommandType          _canCmd;
   struct can_frame        _can_tx_odo;
   struct can_frame        _can_tx_cmd;
   bool _can_extern;
   void sendCommand(CanCommandType cmd);
   int can_TxMsg_init(can_frame* frame, int id, int dlc);
};

#endif