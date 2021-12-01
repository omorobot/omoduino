/**
 * @file r1_driver.h 
 * @brief R1 driver library is for driving OMO-R1 robot along with other OMOROBOT's proprietary sensor modules using CAN transceiver such as MCP2515 
 * For more info, please visit www.omorobot.com 
 *  
 * @License
 * Copyright (c) OMOROBOT INC. All rights reserved. Copyright (c) Kyuhyong You. All rights reserved. 
 * This library is free software; you can redistribute it and/or modify it under the terms of the GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License along with this library; if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 * */
#ifndef _R1_DRIVER_H_
#define _R1_DRIVER_H_

#include <inttypes.h>
#include "r1_command.h"
#include "r1_controller.h"


#define DEFAULT_TURN_W          180 //100

//#define DEBUG_DRIVER              //Uncomment this to print can messages on Serial port
//#define SAME_TAG_REFRESH_EN       //Uncomment this to refresh same tag filter after certain period
class MCP2515;

enum DRIVE_MODE{
    DRIVE_MODE_DEFAULT,
    DRIVE_MODE_LINETRACER        ///Set to line tracer mode when Line sensor available
};
enum REMOTE_MODE{
   REMOTE_NONE,
   REMOTE_SBUS,
   REMOTE_JOY
};
enum DRIVE_DIRECTION{
    DIRECTION_FORWARD,
    DIRECTION_REVERSE
};
enum LINE_FACING{
    FACING_FORWARD,
    FACING_REVERSE
};
enum TURN_DIRECTION{
    TURN_RIGHT,
    TURN_LEFT
};
enum PL_LOAD_UNLOAD{
    PL_LOADING,
    PL_UNLOADING
};

enum TAG_Type{
    TAG_None                = 0,
    TAG_DEPOT               = 0xA0,     //160
    TAG_POU                 = 0xA1,     //161
    TAG_STATION             = 0xA2,     //162
    TAG_APPROACH            = 0xAA,     //170
    TAG_TURN                = 0xB0,     //176
    TAG_LIFT                = 0xB1,     //177
    TAG_TURN_PL             = 0xB2,     //178
    TAG_LOAD_UNLOAD_STOP    = 0xB3,     //179
    TAG_TURN_PL2            = 0xB4,     //180
    TAG_CIN                 = 0xC1,     //193
    TAG_COUT                = 0xC0,     //192
    TAG_SPEED               = 0xE0,     //224
    TAG_SONAR               = 0xE2,     //226
    TAG_READY               = 0xFE      //254
};

typedef struct Tag_Struct{
    uint8_t     bytes[4];
    TAG_Type    type;
} Tag_Struct;

class OMOROBOT_R1
{
   typedef int (R1_Controller::*m_line_control_event)(double);
   typedef int (R1_Controller::*m_speed_control_event)(int, bool);
public:
    
   typedef void (*R1_NewDataClientEvent)(R1_MessageType);
   typedef void (*R1_NewTagReadEvent)(Tag_Struct);
   OMOROBOT_R1();
   OMOROBOT_R1(uint16_t cspin);        //Added to support for different cs pin
   OMOROBOT_R1(MCP2515* mcp2515);

   void     begin(void);
   void     onNewData(R1_NewDataClientEvent cbEvent);
   void     onNewTag(R1_NewTagReadEvent cbEvent);
   void     spin(void);
   void     control_motor_VW(int V, int W);
   void     request_odo();
   void     set_driveMode(R1_VEHICLE_TYPE type, DRIVE_MODE mode);
   void     set_remoteMode(REMOTE_MODE mode);
   void     set_lineoutTime(int ms);
   void     new_can_line(struct can_frame can_rx);
   void     new_can_odo(struct can_frame can_rx);
   void     go(int target_speed);
   void     go(void);
   void     stop();
   void     pause();
   bool     get_go_flag();
   int      get_odo_l();
   int      get_odo_r();
   double   get_linePos();
   int      get_lineout_flag();
   double   get_magnetic_linePos(struct can_frame mag_rx);
   void     set_load_unload_stop();
   //int      get_lineoutTimer();
   //int     can_TxMsg_init(struct can_frame* frame, int id, int dlc);
   void     set_drive_direction(DRIVE_DIRECTION dir, LINE_FACING);
   void     start_turn_odo(TURN_DIRECTION dir, int turn_odo_cnt);
   void     start_turn_timer(PL_LOAD_UNLOAD load_unload, TURN_DIRECTION dir, int speed, int time);
   void     start_turn_timer2(TURN_DIRECTION dir, int speed, int time);
   //void     set_turn_speed(uint16_t turn_W);
   void     set_pl_lift_mode(PL_LIFT_MODE_TYPE mode);
   void     set_v_accel(uint16_t accel);
   void     set_pid_gains(PID_Type pid);
   void     set_turning_speed(int V, int W);
   int      get_target_speed(void);
   void     set_speed(int V);
   void     set_lineout_delay(int ms);
private:
   //typedef void (OMOROBOT_R1::*m_process)(void);
   R1_NewDataClientEvent   _cbDataEvent;
   R1_NewTagReadEvent      _cbTagEvent;
   DRIVE_MODE              _drive_mode;
   R1_VEHICLE_TYPE         _vehicle_type;
   R1_Controller           Controller;

   m_speed_control_event   m_5ms_speed_control;
   m_line_control_event    m_10ms_line_control;
   //m_process               m_turn_process;
   R1_CanBus               CanBus;
   uint64_t                _odoRequest_millis_last;
   uint64_t                _lineDetect_millis_last;            //Last time line detected millis
   bool                    _isLineOut;
   bool                    _go_flag;
   int                     _cmd_speed;
   int                     _goal_V;
   int                     _goal_V_gain;
   int                     _goal_W;
   int                     _v_dir;
   int                     _w_dir;

   int8_t                  _line_pos;
   double                  _line_pos_last;

   int       _lineOut_timer;
   int       _lineOut_timeOut_ms;
   int       _target_speed;          // target speed when go flag is set
   int       _resume_speed;          // target speed when paused

   uint8_t                 _turn_state;
   uint8_t                 _turn_timer_state;
   uint8_t                 _turn_timer_state2;
   uint8_t                 _turn_cmd;
   uint16_t                _turn_odo_cnt;
   int16_t                 _turning_W;
   int16_t                 _turning_V;
   uint16_t                _turn_timer_set;
   uint16_t                _turn_timer_set2;
   uint64_t                _turn_timer_start_millis;
   uint64_t                _turn_timer_start_millis2;
   PL_LOAD_UNLOAD          _load_unload;
   bool                    _is_load_unload_finished;
   bool                    _odo_reset;
   int                     _odo_l;
   int                     _odo_r;

   uint64_t                _5ms_loop_millis_last;
   uint64_t                _loop_control_next_millis;//_10ms_loop_millis_last;

   uint64_t                _100ms_loop_millis_last;
   bool                    _can_rx_extern = false;             //Can rx read performed externally
   uint8_t                 _tag_data_prev[4];
   uint16_t                _same_tag_reset_timer;
   Tag_Struct              _new_tagStr;           //Turn odo count to stop turn
   struct can_frame _canRxMsg;
   REMOTE_MODE             _remote_mode;
   void newCanRxEvent(can_frame can_rx);
   void turn_process_odo(void);
   void turn_process_timer(void);
   void turn_process_timer2(void);
};

#endif