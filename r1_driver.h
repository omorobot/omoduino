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
    /// 기본 구동 제어 모드
    DRIVE_MODE_DEFAULT,
    /// 라인센서가 있는 경우 이 모드로 설정하면 라인 Following 모드로 제어됨
    DRIVE_MODE_LINETRACER        
};

enum REMOTE_MODE{
   REMOTE_NONE,
   REMOTE_SBUS,
   REMOTE_JOY
};

enum DRIVE_DIRECTION{
    /// 속도 명령의 +가 전진 방향인 경우
    DIRECTION_FORWARD,
    /// 속도 명령의 +가 후진 방향인 경우
    DIRECTION_REVERSE
};

enum LINE_FACING{
    /// 라인센서가 진행 방향으로 설치된 경우
    FACING_FORWARD,
    /// 라인센서가 진행 방향에 반대 방향으로 설치된 경우
    FACING_REVERSE
};

enum TURN_DIRECTION{
    /// 좌회전
    TURN_LEFT = 1,
    /// 우회전
    TURN_RIGHT = 2
};

enum PL_LOAD_UNLOAD{
    PL_LOADING,
    PL_UNLOADING
};


/// BMS status
typedef struct BMS_Struct{
    double voltage;
    uint16_t soc;
    uint16_t current_mA;
}BMS_Struct;

/**
 * @brief OMOROBOT Motor driver class
 * 
 */
class OMOROBOT_R1
{
   typedef int (R1_Controller::*m_line_control_event)(double);
   typedef int (R1_Controller::*m_speed_control_event)(int, bool);
public:
   typedef struct TurnCommandStruct{
       /// turning odo count
      int      target_odo_cnt;  
      /// Speed for turning;
      int      speed_v;         
      /// Rotational velocity for turning
      int      speed_w;         
      /// turn odo process state num
      int      state_odo;       
      /// turn timer process state num
      int      state_timer;     
      int      state_timer2;
      uint64_t target_timer_set;
      ///wait timer count for delayed process
      uint16_t wait_cnt;      
      ///Timer ms for turning 
      uint64_t timer_start_millis;  
   }TurnCommandStruct;  
   //typedef void (*R1_NewDataClientEvent)(R1_MessageType);
   //typedef void (*R1_NewTagReadEvent)(const uint8_t[4]);
   OMOROBOT_R1();
   OMOROBOT_R1(uint16_t cspin);        //Added to support for different cs pin
   OMOROBOT_R1(MCP2515* mcp2515);

   void     begin(void);
   //void     onNewData(R1_NewDataClientEvent cbEvent);
   //void     onNewTag(R1_NewTagReadEvent cbEvent);
   void     spin(void);
   void     control_motor_VW(int V, int W);
   void     request_odo();
   void     set_driveMode(R1_VEHICLE_TYPE type, DRIVE_MODE mode);
   void     set_remoteMode(REMOTE_MODE mode);
   void     set_lineoutTime(int ms);
   void     new_can_line(struct can_frame can_rx);
   void     new_can_odo(struct can_frame can_rx);
   void     new_can_bms(struct can_frame can_rx);
   void     go(int target_speed);
   void     go(void);
   void     stop();
   void     pause();
   bool     get_go_flag();
   int      get_odo_l();
   int      get_odo_r();
   double   get_linePos();
   void     set_load_unload_stop();
   void     set_drive_direction(DRIVE_DIRECTION dir, LINE_FACING);
   void     set_uturn_odo(int);
   void     start_turn_degree(int deg, uint16_t speed);
   int      start_turn_odo(int turn_odo_cnt, uint16_t speed);
   void     start_turn_timer(PL_LOAD_UNLOAD load_unload, TURN_DIRECTION dir, int speed, int time);
   void     set_pl_lift_mode(PL_LIFT_MODE_TYPE mode);
   void     set_v_accel(uint16_t accel);
   void     set_pid_gains(PID_Type pid);
   void     set_turning_speed(int V, int W);
   int      get_target_speed(void);
   void     set_speed(int V);
   void     set_lineout_delay(int ms);
   void     set_emergency();
   void     clear_emergency();
   uint16_t     get_bmsSOC(void);
   double       get_voltage(void);
   uint16_t     get_current_mA(void);
   void     send_tagAck(uint8_t);
private:
   //typedef void (OMOROBOT_R1::*m_process)(void);
   //R1_NewDataClientEvent   _cbDataEvent;
   //R1_NewTagReadEvent      _cbTagEvent;
   DRIVE_MODE              _drive_mode;
   R1_VEHICLE_TYPE         _vehicle_type;
   R1_Controller           Controller;

   m_speed_control_event   m_5ms_speed_control;
   m_line_control_event    m_10ms_line_control;
   //m_process               m_turn_process;
   R1_CanBus               CanBus;
   TurnCommandStruct       turn_cmd;
   uint64_t                _odoRequest_millis_last;
   uint64_t                _lineDetect_millis_last;            //Last time line detected millis
   bool                    _isLineOut;
   bool                    _go_flag;
   bool                    _emergency_state;                //Indicate whether emergency is set
   int                     _cmd_speed;
   int                     _goal_V;
   int                     _goal_V_gain;
   int                     _goal_W;
   int                     _v_dir;
   int                     _w_dir;
    int                     _uturn_odo;                 /// Odometry for 180 dgree turn
   double                  _line_pos;
   double                  _line_pos_last;

   int       _lineOut_timer;
   /// Wait time to stop when line out detected
   int       _lineOut_timeOut_ms;    
   /// target speed when go flag is set
   int       _target_speed;          
   /// target speed when paused
   int       _resume_speed;          

   PL_LOAD_UNLOAD          _load_unload;
   bool                    _is_load_unload_finished;
   bool                    _odo_reset;
   int                     _odo_l;
   int                     _odo_r;

   uint64_t                _5ms_loop_millis_last;
   /// _10ms_loop_millis_last;
   uint64_t                _loop_control_next_millis; 

   uint64_t                _100ms_loop_millis_last;
   //Can rx read performed externally
   bool                    _can_rx_extern = false;   
   
   uint16_t                _same_tag_reset_timer;
    //Turn odo count to stop turn
   //Tag_Struct              _new_tagStr;          
   struct can_frame _canRxMsg;
   REMOTE_MODE             _remote_mode;
   BMS_Struct _bms;
   void newCanRxEvent(can_frame can_rx);
   void turn_process_odo(void);
   void turn_process_timer(void);
   void turn_process_timer2(void);
};

#endif