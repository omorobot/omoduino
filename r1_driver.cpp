#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "r1_driver.h"
#include <string.h>

volatile int       _lineOut_timeOut_ms;
//volatile uint8_t   _tag_data_prev[4];
int turn_wait_timer = 0;
/**
 * @brief Get target speed set
 * 
 * @return int mm/s
 */
int OMOROBOT_R1::get_target_speed()
{
   return _target_speed;
}
void OMOROBOT_R1::set_speed(int V)
{
   _target_speed = V;
}
/**
 * @brief Set odometry count for 180 degree turn
 * 
 * @param odo odometry count for uturn
 */
void OMOROBOT_R1::set_uturn_odo(int odo)
{
   this->_uturn_odo = odo;
}

void OMOROBOT_R1::start_turn_degree(int degree, uint16_t speed)
{
   if(degree > 0) {
      this->turn_cmd.speed_w = speed;
      this->turn_cmd.target_odo_cnt = degree * this->_uturn_odo / 180;
   } else {
      this->turn_cmd.speed_w = -speed;
      this->turn_cmd.target_odo_cnt = - degree * this->_uturn_odo / 180;
   }
   this->turn_cmd.state_odo = 1;
}

/**
 * @brief Initiate turn process with odometer count
 * 
 * @param turn_odo_cnt (+) turn_odo_cnt turn to left vs (-) turn_odo_cnt turns to right
 * @param speed speed is for designated turning speed
 * @return int 
 */
int OMOROBOT_R1::start_turn_odo(int turn_odo_cnt, uint16_t speed)
{
   if(turn_odo_cnt > 0) {              //Positive turn odo cnt means turn to left
      this->turn_cmd.speed_w = speed;
      this->turn_cmd.target_odo_cnt = turn_odo_cnt;
   } else if(turn_odo_cnt < 0) {       //Negative turn odo cnt means turn to right
      this->turn_cmd.speed_w = -speed;
      this->turn_cmd.target_odo_cnt = -turn_odo_cnt;
   } else {
      return 1;
   }
   // _turn_odo_cnt = turn_odo_cnt;
   // _turn_state = 1;
   this->turn_cmd.state_odo = 1;
   return 0;
   //this->m_turn_process = &this->turn_process_odo;
}

/**
 * @brief Initiate turn process with timer for PL-1500 type vehicle
 * 
 * @param load_unload 
 * @param dir Turn direction
 * @param speed Speed for motor
 * @param time Time to turn
 */
void OMOROBOT_R1::start_turn_timer(PL_LOAD_UNLOAD load_unload, TURN_DIRECTION dir, int speed, int time)
{
   this->turn_cmd.state_timer = 1;
   _load_unload = load_unload;
   if(dir == TURN_RIGHT) {
      turn_cmd.speed_w = 180;
   } else if(dir == TURN_LEFT) {
      turn_cmd.speed_w = -180;
   }
   turn_cmd.speed_v = speed;
   turn_cmd.target_timer_set = time;
   //_turn_timer_set = time;
   Serial.print("Set turn timer:"); Serial.println((uint16_t)turn_cmd.target_timer_set);
   _is_load_unload_finished = false;
   //this->m_turn_process = &this->turn_process_timer;
}
/**
 * @brief Turn process with odometry count (More precise)
 * */
void OMOROBOT_R1::turn_process_odo(void)
{
   switch(this->turn_cmd.state_odo){
   case 0:
      //Do nothing here  
      break;
   case 1:   //Stop the vehicle
      _go_flag = false;
      _goal_W = 0;
      if(_cmd_speed == 0) {
         _odo_reset = true;
         if(this->turn_cmd.wait_cnt++ > 200) {
            turn_cmd.state_odo = 2;
            turn_cmd.wait_cnt = 0;
            _odo_reset = false;
         }
      }
      break;
   case 2:
      if(abs(_odo_l) < 50) {   //Check odometry reset
         _odo_reset = false;
         turn_cmd.state_odo = 3;
      }
      break;
   case 3:
      _goal_W = 0;
      if(turn_wait_timer++ > 100) {
         turn_cmd.state_odo = 4;
         Serial.println("TURN Start");
      }
   break;
   case 4:               //Now start to turn
      _goal_V = turn_cmd.speed_v;
      _goal_W = turn_cmd.speed_w;
      turn_cmd.state_odo = 5;
      break;
   case 5:                     
      // Serial.print("ODO L:");
      // Serial.println(_odo_l); 
      if(abs(_odo_l) > turn_cmd.target_odo_cnt) {  //Finish turn at _turn_odo_cnt set
         Serial.println("TURN ODO met GOAL");
         _goal_W = 0;
         _goal_V = 0;
         turn_cmd.state_odo = 6;
         turn_cmd.wait_cnt = 0;
      }
      break;
   case 6:
      if(turn_cmd.wait_cnt++> 100) {
         turn_cmd.state_odo = 7;
      }
      break;
   case 7:
      _go_flag = true;
      turn_cmd.state_odo = 0;
   break;
   }
}
/**
 * @brief Turn process with timer2 value
 * 
 */
void OMOROBOT_R1::turn_process_timer2(void)
{
   switch(turn_cmd.state_timer2){
   case 0:
      //Do nothing here  
      break;
   case 1:   //First stop the vehicle
      _go_flag = false;
      if(_cmd_speed == 0) {
         //CanBus.set_pl_lift_mode(PL_LIFT_DOWN);
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer2 = 2;
         Serial.print("Turn state2:"); Serial.println(turn_cmd.state_timer2);
      }
      break;
   case 2:     //Wait for 2.5 sec
      _goal_W = 0;
      _goal_V = 0;
      if( (millis() - turn_cmd.timer_start_millis) > 2500) {
         turn_cmd.state_timer2 = 3;
         turn_cmd.timer_start_millis = millis();
      }
      break;
   case 3:     //Set the steering to the turn angle
      _goal_W = turn_cmd.speed_w;
      _goal_V = turn_cmd.speed_v;
      if( (millis() - turn_cmd.timer_start_millis) > 2500) {
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer2 = 4;
         Serial.print("Turn state2:"); Serial.println(turn_cmd.state_timer2);
      }
      break;
   case 4:     //Start moving the wheel for designated turn timer set
      _goal_W = turn_cmd.speed_w;
      _goal_V = turn_cmd.speed_v;
      if( (millis() - turn_cmd.timer_start_millis) > turn_cmd.target_timer_set) {   
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer2 = 5;
         Serial.print("Turn state2:"); Serial.println(turn_cmd.state_timer2);
      }
   break;
   case 5:     //Finished turn so stop motion
      _goal_W = 0;
      _goal_V = 0;
      if( (millis() - turn_cmd.timer_start_millis) > 2500) {   
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer2 = 6;
      }
   break;
   case 6:
      _go_flag = true;
      turn_cmd.state_timer2 = 0;
      break;
   }
}
/**
 * @brief Turn process with timer
 * 
 */
void OMOROBOT_R1::turn_process_timer(void)
{
   switch(turn_cmd.state_timer){
   case 0:
      //Do nothing here  
      break;
   case 1:   //First stop the vehicle
      _go_flag = false;
      if(_cmd_speed == 0) {
         //CanBus.set_pl_lift_mode(PL_LIFT_DOWN);
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer = 2;
      }
      break;
   case 2:     //Wait for 2.5 sec
      _goal_W = 0;
      _goal_V = 0;
      if( (millis() - turn_cmd.timer_start_millis) > 2500) {
         turn_cmd.state_timer = 3;
         turn_cmd.timer_start_millis = millis();
      }
      break;
   case 3:     //Set the steering to the turn angle
      _goal_W = turn_cmd.speed_w;
      _goal_V = 0;
      if( (millis() - turn_cmd.timer_start_millis) > 2500) {
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer = 4;
      }
      break;
   case 4:     //Start moving the wheel for designated turn timer set
      _goal_W = turn_cmd.speed_w;
      _goal_V = turn_cmd.speed_v;
      if( (millis() - turn_cmd.timer_start_millis) > turn_cmd.target_timer_set) {   
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer = 5;
      }
   break;
   case 5:     //Finished turn so stop motion
      _goal_W = 0;
      _goal_V = 0;
      if((millis() - turn_cmd.timer_start_millis) > 2500) {
         if(_load_unload == PL_LOADING) {
            CanBus.set_pl_lift_mode(PL_LIFT_DOWN);
            turn_cmd.state_timer = 6;
         } else {
            turn_cmd.state_timer = 7;
         }
         turn_cmd.timer_start_millis = millis();
      }
   break;
   case 6:
      _goal_W = 0;
      _goal_V = 0;
      if((millis() - turn_cmd.timer_start_millis) > 10000) {   //Wait for 10 seconds   for lift down position
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer = 7;
         Serial.print("Start lift up/down"); Serial.println(turn_cmd.state_timer);
      }
   break;
   case 7:
      _goal_V = - 500;        //Move backwards
      _goal_W = 0;
      if(_is_load_unload_finished) {
         turn_cmd.state_timer = 8;
         turn_cmd.timer_start_millis = millis();
         Serial.print("Load unload finished tag"); Serial.println(turn_cmd.state_timer);
      } else if((millis() - turn_cmd.timer_start_millis) > 20000) {
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer = 8;
         Serial.print("State 8 time_out:"); Serial.println(turn_cmd.state_timer);
      }
   break;
   case 8:
      _goal_V = 0;        //Stop vehicle 
      _goal_W = 0;
      if(_load_unload == PL_LOADING) {             // Lift up for loading 
         CanBus.set_pl_lift_mode(PL_LIFT_UP);
         //Serial.print("PL_LIFT_UP:"); Serial.println(_turn_timer_state);
      }else if(_load_unload == PL_UNLOADING) {     // Lift down for unloading
         CanBus.set_pl_lift_mode(PL_LIFT_DOWN);
         //Serial.print("PL_LIFT_DOWN:"); Serial.println(_turn_timer_state);
      }
      if((millis()-turn_cmd.timer_start_millis) > 12000) {
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer = 9;
         Serial.print("State:"); Serial.println(turn_cmd.state_timer);
      }
   break;
   case 9:
      _goal_V = 500;
      _goal_W = 0;
      if((millis() - turn_cmd.timer_start_millis) > 5000) {
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer = 10;
         Serial.print("State:"); Serial.println(turn_cmd.state_timer);
      }
   break;
   case 10:
      _goal_V = 0;
      _goal_W = 0;
      if((millis() - turn_cmd.timer_start_millis) > 2000) {
         turn_cmd.timer_start_millis = millis();
         turn_cmd.state_timer = 0;
         _go_flag = true;
         Serial.print("Turn timer State finished"); Serial.println(turn_cmd.state_timer);
      }
   case 11:

   break;
   }
}
/**
*   @brief Initialize MCP2515 with default CS pin for Arduino Uno
*/
OMOROBOT_R1::OMOROBOT_R1() {
   CanBus = R1_CanBus();
   CanBus.onNewCanRx(this, &this->newCanRxEvent);
   _drive_mode = DRIVE_MODE_DEFAULT;
}
/**
*   @brief Initialize MCP2515 with different CS pin for Arduino Mega 2560 board
*/
OMOROBOT_R1::OMOROBOT_R1(uint16_t cspin) {
   CanBus = R1_CanBus(cspin);
   CanBus.onNewCanRx(this, &this->newCanRxEvent);
   _drive_mode = DRIVE_MODE_DEFAULT;
}
/**
* @brief Initialize with MCP2515 object as external reference
*/
OMOROBOT_R1::OMOROBOT_R1(MCP2515* mcp2515) {
   CanBus = R1_CanBus(mcp2515);
   _drive_mode = DRIVE_MODE_DEFAULT;
   _can_rx_extern = true;    //CanRx is scanned in external reference so no need to scan in the loop
}
/**
* @brief Begin initialize items
*/
void OMOROBOT_R1::begin() {
  
   if(!_can_rx_extern) {
      CanBus.begin_bus();
   }
   _go_flag = false;
   _target_speed = 0;
   _goal_V = 0;
   _goal_W = 0;
   _goal_V_gain = 0;
   _cmd_speed = 0;
   _isLineOut = false;
   _lineOut_timeOut_ms = 0;
   _lineOut_timer = 0;
   turn_cmd.target_odo_cnt = 0;
   turn_cmd.target_timer_set = 0;

   turn_cmd.speed_w = 0;
   turn_cmd.speed_v = 0;
   turn_cmd.state_odo = 0;
   turn_cmd.state_timer = 0;
   turn_cmd.state_timer2 = 0;
   turn_cmd.timer_start_millis = 0;
   turn_cmd.wait_cnt = 0;
   _odoRequest_millis_last = millis();
   this->_emergency_state = false;
}
/**
 * @brief Register callback function for new data event
 * 
 * @param cbEvent 
 */
//void OMOROBOT_R1::onNewData(R1_NewDataClientEvent cbEvent){ _cbDataEvent = cbEvent;}
/**
 * @brief Register callback function for new TAG message event
 * 
 * @param cbEvent 
 */
//void OMOROBOT_R1::onNewTag(R1_NewTagReadEvent cbEvent){ _cbTagEvent = cbEvent;}
/**
* @brief R1 main loop
*/
void OMOROBOT_R1::spin() {
   if(!_can_rx_extern) {
      CanBus.scan();
   }
   if(millis() - _5ms_loop_millis_last > 4) {
      if(this->m_5ms_speed_control) {
         _cmd_speed = (Controller.*this->m_5ms_speed_control)(_cmd_speed, _go_flag);
         _goal_V = _cmd_speed;
      }
      _5ms_loop_millis_last = millis();
   }
   if(millis()-_odoRequest_millis_last > 9) {
      CanBus.request_odo(_odo_reset);
      if(turn_cmd.state_timer > 0) {
         this->turn_process_timer();
      }
      if(this->turn_cmd.state_odo > 0) {
         this->turn_process_odo();
      }
      _odoRequest_millis_last = millis();

   }
   if(millis() >= _loop_control_next_millis) { //-_10ms_loop_millis_last > 9) {
      if(this->m_10ms_line_control) {
         if(_go_flag) {
            _goal_W = (Controller.*this->m_10ms_line_control)(_line_pos);
         } else {
            if(turn_cmd.state_odo == 0) {
               _goal_W = 0;
            }
         } 
         if(turn_cmd.state_timer2 > 0) {
            this->turn_process_timer2();
         }
         if(_vehicle_type == VEHICLE_TYPE_PL153) {
            CanBus.cmd_pl_dac_angle(_goal_V*_v_dir, _goal_W*_w_dir);  
         } else {
            // For R1 send V and W
            if(this->_remote_mode == REMOTE_NONE){
               //CanBus.cmd_VW((_goal_V+_goal_V_gain)*_v_dir, _goal_W*_w_dir);
               CanBus.cmd_VW(_goal_V*_v_dir, _goal_W*_w_dir);
            }
         }
      }
      _loop_control_next_millis+=5;
      //_10ms_loop_millis_last = millis();
   }
   if(millis() - _100ms_loop_millis_last > 99) {
   #ifdef SAME_TAG_REFRESH_EN
      if(_same_tag_reset_timer>0) {
         if(_same_tag_reset_timer > 199) {
            _same_tag_reset_timer-= 100;
         } else {
            _tag_data_prev[0] = 0;
            _tag_data_prev[1] = 0;
            _tag_data_prev[2] = 0;
            _tag_data_prev[3] = 0;
            _same_tag_reset_timer = 0;
         }
      }
   #endif
      _100ms_loop_millis_last = millis();
   }
}
/**
 * @brief Set wait timer in ms to stop vehicle when there is no line detected
 * 
 * @param ms 
 */
void OMOROBOT_R1::set_lineout_delay(int ms)
{
   _lineOut_timeOut_ms = ms;
}
/**
*  @brief Process line position message from can bus
*/
void OMOROBOT_R1::new_can_line(struct can_frame can_rx)
{
   switch(can_rx.data[0]) {
   case 1:   //Line detect
   {
      _isLineOut = false;
      int8_t line_pos = can_rx.data[1];
      _line_pos = (double)line_pos;
      _lineDetect_millis_last = millis();    //Update line detection time
      _lineOut_timer = 0;     //reset lineout timer
      uint8_t tagAck = can_rx.data[4];
      tagAck += can_rx.data[5];
      tagAck += can_rx.data[6];
      tagAck += can_rx.data[7];
      tagAck = ~tagAck;
   }
      break;
   case 2:   //No line
      _isLineOut = true;
      if(_lineOut_timeOut_ms > 0) {
         if( (millis() - _lineDetect_millis_last) > _lineOut_timeOut_ms) {
            _go_flag = false;    //Stop the line tracer
         }
      }
      //_cbDataEvent(R1MSG_LINEOUT);
      break;
   }
}
/**
* @brief Process odoMsg from can bus
*/
void OMOROBOT_R1::new_can_odo(struct can_frame can_rx)
{
   if(can_rx.data[0] == 0x02) {
      _odo_r = (can_rx.data[1]|(can_rx.data[2]<<8));
      _odo_l = (can_rx.data[3]|(can_rx.data[4]<<8));
   } else if(can_rx.data[0] == 0x03) {
      _odo_r = (can_rx.data[1]|(can_rx.data[2]<<8)|(can_rx.data[3]<<16));
      _odo_l = (can_rx.data[4]|(can_rx.data[5]<<8)|(can_rx.data[6]<<16));
   }
}

void OMOROBOT_R1::new_can_bms(struct can_frame can_rx)
{
   this->_bms.voltage = (double)(can_rx.data[0]|(can_rx.data[1]<<8)) / 10.0;
   this->_bms.soc = can_rx.data[2]|(can_rx.data[3]<<8);
   this->_bms.current_mA = can_rx.data[4]|(can_rx.data[5]<<8);
}

uint16_t     OMOROBOT_R1::get_bmsSOC(void){
   return this->_bms.soc;
}
double       OMOROBOT_R1::get_voltage(void){
   return this->_bms.voltage;
}
uint16_t     OMOROBOT_R1::get_current_mA(void){
   return this->_bms.current_mA;
}
void        OMOROBOT_R1::send_tagAck(uint8_t ack) {
   this->CanBus.send_tagAck(ack);
}
/**
 * @brief Process new can message event from CAN-bus module
 * 
 * @param _canRxMsg 
 */
void OMOROBOT_R1::newCanRxEvent(struct can_frame _canRxMsg)
{
   int senderID = (_canRxMsg.can_id>>4);
   int dlc = _canRxMsg.can_dlc;
   if(senderID == 0x02) { //From LINE sensor
      switch(_canRxMsg.data[0]) {
      case 1:   //Line detect
         _isLineOut = false;
         _line_pos = (double)_canRxMsg.data[1]; //was int8_t
         _lineDetect_millis_last = millis();    //Update line detection time
         _lineOut_timer = 0;
         //_cbDataEvent(R1MSG_LINEPOS);
         break;
      case 2:   //No line
         _isLineOut = true;
         if(_lineOut_timeOut_ms > 0) {
            if( (millis() - _lineDetect_millis_last) > _lineOut_timeOut_ms) {
               _go_flag = false;
            }
         } 
         //_cbDataEvent(R1MSG_LINEOUT);
         break;
      }
   } else if(senderID == 0x4) {
      if(_canRxMsg.data[0] == 0x02) {
         _odo_r = (_canRxMsg.data[1]|(_canRxMsg.data[2]<<8));
         _odo_l = (_canRxMsg.data[3]|(_canRxMsg.data[4]<<8));
         //_cbDataEvent(R1MSG_ODO);
      }
   } else if(senderID == 0x06) {   //From conveyor
      
   }
}
/**
 * @brief Control motor driver with desired V and W
 * 
 * @param V vehicle speed in mm/s
 * @param W vehicle speed in mrad/s
 */
void OMOROBOT_R1::control_motor_VW(int V, int W) {
   V = V*_v_dir;
   W = W*_w_dir;
   if(this->_drive_mode == DRIVE_MODE_LINETRACER) {
      if(this->_remote_mode != REMOTE_NONE) {
         CanBus.cmd_VW(V, W);
      }
   } else {
      CanBus.cmd_VW(V, W);
   }
}
/**
 * @brief Request odometry to motor driver
 * 
 */
void OMOROBOT_R1::request_odo() {
   CanBus.request_odo(_odo_reset);
}
/**
 * @brief Set motor drive mode
 * 
 * @param type Supporting vehicle type R1 or PL153
 * @param mode Speed control or line control mode
 */
void OMOROBOT_R1::set_driveMode(R1_VEHICLE_TYPE type, DRIVE_MODE mode)
{
   _vehicle_type = type;
   CanBus.set_vehicle_type(type);
   _drive_mode = mode;
   if(_drive_mode == DRIVE_MODE_LINETRACER) {
      Serial.println("Set Line tracer mode");
      Controller.reset_pid_line();
      _goal_V = 0;
      _goal_W = 0;
      _cmd_speed = 0;
      if(_vehicle_type == VEHICLE_TYPE_R1) {
         this->m_5ms_speed_control = &Controller.speed_control;
         this->m_10ms_line_control = &Controller.line_control_vw;
         set_turning_speed(0, 100);
      } else if(_vehicle_type == VEHICLE_TYPE_PL153) {
         set_turning_speed(500, 180);
         this->m_5ms_speed_control = &Controller.speed_control;
         this->m_10ms_line_control = &Controller.line_control_angle;
      }
      
      _5ms_loop_millis_last = millis();
      _loop_control_next_millis = millis() + 5;
      //_10ms_loop_millis_last = millis();
   }
}
/**
 * @brief Set remote mode
 * 
 * @param mode 
 */
void OMOROBOT_R1::set_remoteMode(REMOTE_MODE mode)
{
   this->_remote_mode = mode;
}

/**
 * @brief Sets turning speed and rate of change of direction
 * R1 type vehicle normally turns with V = 0 and W only;
 * @param V 
 * @param W 
 */
void OMOROBOT_R1::set_turning_speed(int V, int W)
{
   turn_cmd.speed_v = V;
   turn_cmd.speed_w = W;
}
/**
 * @brief Sets drive direction of the vehicle 
 * 
 * @param dir Move forward ro reversed direction
 * @param line Whether the line sensor is facing forward or reversed
 */
void OMOROBOT_R1::set_drive_direction(DRIVE_DIRECTION dir, LINE_FACING line)
{
   if(dir == DIRECTION_FORWARD) {
      _v_dir = 1;
   }else if(dir == DIRECTION_REVERSE) {
      _v_dir = -1;
   }
   if(line == FACING_FORWARD) {
      _w_dir = 1;
   }else if(line == FACING_REVERSE) {
      _w_dir = -1;
   }
}
/**
 * @brief Set time to stop when lineout detected in ms
 * 
 * @param ms Desired time to stop the vehicle
 */
void OMOROBOT_R1::set_lineoutTime(int ms) { _lineOut_timeOut_ms = ms; }
/**
 * @brief Set PL153 vehicle's lift mode
 * 
 * @param mode 
 */
void OMOROBOT_R1::set_pl_lift_mode(PL_LIFT_MODE_TYPE mode) {
   CanBus.set_pl_lift_mode(mode);
}
/**
 * @brief Set desired acceleration for controlling the vehicle
 * 
 * @param accel 
 */
void OMOROBOT_R1::set_v_accel(uint16_t accel) {
   Controller.set_v_accel(accel);
}
/**
 * @brief Set desired PID gains setting
 * 
 * @param pid 
 */
void OMOROBOT_R1::set_pid_gains(PID_Type pid) {
   Controller.set_pid_gain_line(pid);
}

/**
 * @brief Start vehicle with target speed
 * 
 * @param target_speed 
 */
void OMOROBOT_R1::go(int target_speed)
{
   if(target_speed) {
      _lineOut_timer = 0;
      Controller.set_target_v(target_speed);
      Serial.print("R1 GO:");
      Serial.println(target_speed);
      _target_speed = target_speed;
      _resume_speed = target_speed;
   }
   if(!this->_emergency_state) {
      _go_flag = true;
   } else {
      Serial.println("NO GO: Emergency");
   }
}

/**
 * @brief Resume vehicle motion when paused. 
 * If vehicle is stopped, calling this wouldn't start the vehicle
 * 
 */
void OMOROBOT_R1::go(void)
{
   _lineOut_timer = 0;
   _target_speed = _resume_speed;
   Controller.set_target_v(_target_speed);
   if(!this->_emergency_state) {
      _go_flag = true;
   } else {
      Serial.println("NO GO: Emergency");
   }
}

/**
 * @brief Completely stop the vehicle. 
 * reset _target_speed to 0, _go_flag = false
 * 
 */
void OMOROBOT_R1::stop()
{
   _target_speed = 0;
   turn_cmd.state_timer2 = 0;
   turn_cmd.state_timer = 0;
   _go_flag = false;
   turn_cmd.state_timer = 0;
   turn_cmd.state_odo = 0;
}

/**
 * @brief Only reset target speed to 0 and wait for go()
 * 
 */
void     OMOROBOT_R1::pause()       {  
   _target_speed = 0; 
   //Serial.println("R1 paused");
   Controller.set_target_v(_target_speed);
}

bool     OMOROBOT_R1::get_go_flag() {  return _go_flag;}
int      OMOROBOT_R1::get_odo_l()   {  return _odo_l;}
int      OMOROBOT_R1::get_odo_r()   {  return _odo_r;}
double   OMOROBOT_R1::get_linePos() {  return _line_pos;}



void OMOROBOT_R1::set_load_unload_stop()
{
   _is_load_unload_finished = true;
}

void OMOROBOT_R1::set_emergency()
{
   this->_emergency_state = true;
}
void OMOROBOT_R1::clear_emergency()
{
   this->_emergency_state = false;
}