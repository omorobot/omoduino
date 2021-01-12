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
#include "line_detector.h"


#define DEFAULT_TURN_W          180 //100

#define LINE_EDGE_SPEED        200
#define LINE_EDGE_POS_H          6.0
#define LINE_EDGE_POS_L          4.0

#define LINE_METHOD_0

//#define DEBUG_DRIVER              //Uncomment this to print can messages on Serial port
//#define SAME_TAG_REFRESH_EN       //Uncomment this to refresh same tag filter after certain period
class MCP2515;

enum DRIVE_MODE{
   DRIVE_MODE_DEFAULT,
   DRIVE_MODE_LINETRACER,        ///Set to line tracer mode when Line sensor available
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
enum LINE_DETECTOR_POSITION{
   DETECTOR_FRONT,
   DETECTOR_REAR
};
enum LINE_DETECTOR_TYPE{
   DETECTOR_TYPE_MAGNETIC = 0,
   DETECTOR_TYPE_OAGV = 1
};

enum TAG_Type{
   TAG_None                = 0,
   TAG_DEPOT               = 0xA0,     //160
   TAG_POU                 = 0xA2,     //162
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
   void     new_can_line(struct can_frame can_rx, LINE_DETECTOR_POSITION);
   void     new_can_odo(struct can_frame can_rx);
   void     new_can_tag(struct can_frame can_rx);
   void     go(int target_speed);
   void     go(void);
   void     stop();
   void     pause();
   bool     get_go_flag();
   int      get_odo_l();
   int      get_odo_r();
   double   get_linePos();
   double   get_linePos(LINE_DETECTOR_POSITION);
   void     set_load_unload_stop();
   void     set_drive_direction(DRIVE_DIRECTION dir, LINE_FACING);
   void     select_line_detector(LINE_DETECTOR_POSITION);
   void     set_line_detectorType(LINE_DETECTOR_TYPE);
   void     start_turn_odo(TURN_DIRECTION dir, int turn_odo_cnt);
   void     start_turn_timer(PL_LOAD_UNLOAD load_unload, TURN_DIRECTION dir, int speed, int time);
   void     start_turn_timer2(TURN_DIRECTION dir, int speed, int time);
   void     stop_turn(void);
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
   LINE_DETECTOR           line_detector_front;
   LINE_DETECTOR           line_detector_rear;
   LINE_DETECTOR_POSITION  _current_line_detector_position;
   m_speed_control_event   m_5ms_speed_control;
   m_line_control_event    m_10ms_line_control;
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

   double                  _line_pos;               //was int8_t
   double                  _line_pos_last;
#ifdef LINE_METHOD_2
      // yoon
   int                     _steer_left_flag;
   int                     _steer_right_flag;
   int                     _steer_flag;
   int                     _steer_dir;             //1: cw, -1:ccw
   double                  _line_pos_bef;
   double                  _line_pos_delta;    
   int                     _line_pos_dir;
   double                  _steer_gain;
   double                  _line_gain;
   uint64_t                _1ms_millis;
   int                     _steer_timer;
#endif
   int                     _lineOut_timer;
   int                     _lineOut_timeOut_ms;
   int                     _target_speed;          // target speed when go flag is set
   int                     _resume_speed;          // target speed when paused


   //uint8_t                 _turn_state;
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
   bool                    _can_rx_extern = false;    //Can rx read performed externally
   uint8_t                 _tag_data_prev[4];
   uint16_t                _same_tag_reset_timer;
   Tag_Struct              _new_tagStr;               //Turn odo count to stop turn
   struct can_frame _canRxMsg;
   REMOTE_MODE             _remote_mode;
   LINE_DETECTOR_TYPE      _line_detector_type;
   void newCanRxEvent(can_frame can_rx);
   void turn_process_odo(void);
   void turn_process_timer(void);
   void turn_process_timer2(void);
   void  process_magnetic_line_sensor(LINE_DETECTOR*, uint16_t, LINE_DETECTOR_POSITION);

};

#endif