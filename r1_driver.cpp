#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include "r1_driver.h"

struct can_frame _canRxMsg;
struct can_frame _canTxMsg_Motor;
struct can_frame _canTxMsg_Odo;

//#define DEBUG_CONTROL     // Un-comment to debug control output

double    _pid_line_p = 36.0;
double    _pid_line_i = 0.015;
double    _pid_line_d = 1.0;

int8_t    _line_pos = 0;
int       _line_error_accum = 0;
int       _line_error_prev = 0;
const int _line_error_accum_max = 250;
const int _line_control_out_max = 300;

int       _lineOut_timer;
int       _lineOut_timeOut_ms;
int       _target_speed;          // target speed when go flag is set
int       _resume_speed;          // target speed when paused
int       _goal_V;                // Velocity(mm/s) to be sent to Motor via CAN
int       _goal_W;                // W(mrad/s) to be sent to Motor via CAN
int       _cmd_speed;             // commanded speed finally set to goal_V
bool      _go_flag;               // Set true to start linetracer
int       _v_dir = 1;             // V forward = 1, reverse = -1
int       _w_dir = 1;             // Line sensor fwd = 1, rear = -1
uint8_t   _turn_state = 0;
uint8_t   _turn_cmd;
uint16_t  _turn_odo_cnt;
uint16_t  _turn_W = DEFAULT_TURN_W; //Turn speed when start to turn;
bool      _odo_reset;
int       _odo_l;
int       _odo_r;

void speed_control(void);
void line_control(void);
void turn_process(void);

void speed_control(void)
{
  if(_go_flag) {
    if(_cmd_speed < _target_speed) {
      _cmd_speed++;
    } else if(_cmd_speed > _target_speed) {
      _cmd_speed--;
    }
  } else {
    if(_cmd_speed > 1) {
      _cmd_speed--;
    } else if(_cmd_speed < -1) {
      _cmd_speed++;
    } else {
      _cmd_speed = 0;
    }
  }
}

void line_control(void)
{
  _goal_V = _cmd_speed;
  if(_go_flag) {
    int error = _line_pos;
    _line_error_accum += error;
    int error_d = error - _line_error_prev;
    _line_error_prev = error;
    if(_line_error_accum > _line_error_accum_max) _line_error_accum = _line_error_accum_max;
    else if(_line_error_accum < -_line_error_accum_max) _line_error_accum = -_line_error_accum_max;
    int output = _pid_line_p * error + _line_error_accum*_pid_line_i + error_d * _pid_line_d;
    if(output > _line_control_out_max) output = _line_control_out_max;
    else if(output < -_line_control_out_max) output = -_line_control_out_max;
    _goal_W = output;
#ifdef DEBUG_CONTROL   
    Serial.print("LINE: error=");
    Serial.print(error);
    Serial.print(" output=");
    Serial.print(output);
    Serial.print(" I=");
    Serial.print(_line_error_accum);
    Serial.println();
#endif
  } else {
    _goal_W = 0;
  }
  turn_process();
}

void turn_process(void)
{
  switch(_turn_state){
  case 0:
    //Do nothing here  
    break;
  case 1:   //Stop the vehicle
    _go_flag = false;
    if(_cmd_speed == 0) {
      _turn_state = 2;
    }
    break;
  case 2:
    _odo_reset = true;
    if(_odo_l == 0) {   //Check odometry reset
      _turn_state = 3;
    }
    break;
  case 3:               //Now start to turn
    if(_turn_cmd == 0) {    //For right turn
      _goal_W = -_turn_W;
    } else if(_turn_cmd == 1) { //For left turn
      _goal_W = _turn_W;
    }
    break;
  }
}
/**
*   @brief Initialize MCP2515 with default CS pin for Arduino Uno
*/
OMOROBOT_R1::OMOROBOT_R1() {
  _mcp2515 = new MCP2515(10);
  _drive_mode = R1DRV_DefaultMode;
}
/**
*   @brief Initialize MCP2515 with different CS pin for Arduino Mega 2560 board
*/
OMOROBOT_R1::OMOROBOT_R1(uint16_t cspin) {
  _mcp2515 = new MCP2515(cspin);
  _drive_mode = R1DRV_DefaultMode;
}
/**
* @brief Initialize with MCP2515 object as external reference
*/
OMOROBOT_R1::OMOROBOT_R1(MCP2515* mcp2515) {
  _mcp2515 = mcp2515;
  _drive_mode = R1DRV_DefaultMode;
  _can_rx_extern = true;
}
/**
* @brief Begin initialize items
*/
void OMOROBOT_R1::begin() {
  
  if(!_can_rx_extern) {
    SPI.begin();
    _mcp2515->reset();
    _mcp2515->setBitrate(CAN_500KBPS);
    _mcp2515->setNormalMode();
  }
  can_TxMsg_init(&_canTxMsg_Motor, 0x4, 8);
  can_TxMsg_init(&_canTxMsg_Odo, 0x4, 8);
  _odoRequest_millis_last = millis();
}

void OMOROBOT_R1::onNewData(R1_NewDataClientEvent cbEvent)
{
  _cbDataEvent = cbEvent;
}
void OMOROBOT_R1::onNewTag(R1_NewTagReadEvent cbEvent)
{
  _cbTagEvent = cbEvent;
}
/**
* @brief R1 main loop
*/
void OMOROBOT_R1::spin() {
  if(!_can_rx_extern) {
    if (_mcp2515->readMessage(&_canRxMsg) == MCP2515::ERROR_OK) {
      
      int senderID = (_canRxMsg.can_id>>4);
      int dlc = _canRxMsg.can_dlc;
  #ifdef DEBUG_DRIVER      
      Serial.print(_canRxMsg.can_id, HEX); // print ID
      Serial.print(" "); 
      Serial.print(_canRxMsg.can_dlc, HEX); // print DLC
      Serial.println();
      
      for (int i = 0; i<_canRxMsg.can_dlc; i++)  {  // print the data
        Serial.print(_canRxMsg.data[i],HEX);
        Serial.print(" ");
      }
      Serial.println();
  #endif
      if(senderID == 0x02) { //From LINE sensor
        switch(_canRxMsg.data[0]) {
          case 1:   //Line detect
            _isLineOut = false;
            _line_pos = (int8_t)_canRxMsg.data[1];
            //Serial.print(_line_pos);
            //Serial.println();
            _lineDetect_millis_last = millis();    //Update line detection time
            _lineOut_timer = 0;
  #ifdef DEBUG_DRIVER
            Serial.print("LINE POS: ");
            Serial.print(_line_pos);
            Serial.println();
  #endif
            _cbDataEvent(R1MSG_LINEPOS);
            break;
          case 2:   //No line
            _isLineOut = true;
            _lineOut_timer = millis() - _lineDetect_millis_last;
            if(_lineOut_timeOut_ms > 0) {
              if(_lineOut_timer > _lineOut_timeOut_ms) {
                _go_flag = false;     //Stop the line tracer
              }
            }
            
            _cbDataEvent(R1MSG_LINEOUT);
            break;
        }
      } else if(senderID == 0x4) {
        if(_canRxMsg.data[0] == 0x02) {
          _odo_r = (_canRxMsg.data[1]|(_canRxMsg.data[2]<<8));
          _odo_l = (_canRxMsg.data[3]|(_canRxMsg.data[4]<<8));
          _cbDataEvent(R1MSG_ODO);
        }
  #ifdef DEBUG_DRIVER
        Serial.print("ODO: L= ");
        Serial.print(_odo_l);
        Serial.print(" R= ");
        Serial.print(_odo_r);
        Serial.println();
  #endif
      } else if(senderID == 0x06) {   //From conveyor
        
      }
    }
  }
  if(_3ms_loop) {
    if(millis() - _3ms_loop_millis_last > 2) {
      _3ms_loop();
      _3ms_loop_millis_last = millis();
    }
  }
  if(millis()-_odoRequest_millis_last > 9) {
      request_odo();
      _odoRequest_millis_last = millis();
  }
  if(_10ms_loop) {
    if(millis()-_10ms_loop_millis_last > 9) {
      _10ms_loop();
      control_motor_VW(_goal_V, _goal_W);
      _10ms_loop_millis_last = millis();
    }
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
*  @brief Process line position message from can bus
*/
void OMOROBOT_R1::new_can_line(struct can_frame can_rx)
{
  uint8_t same_tag_cnt = 0;
  int i;
  switch(can_rx.data[0]) {
  case 1:   //Line detect
    _isLineOut = false;
    _line_pos = (int8_t)can_rx.data[1];
    _lineDetect_millis_last = millis();    //Update line detection time
    _lineOut_timer = 0;
    //Check if tag data is new one
    for(i =0; i<4; i++) {
      if(_tag_data_prev[i]!=can_rx.data[4+i]) {
        same_tag_cnt++;
      }
    }
    if(same_tag_cnt>0){
      Serial.println(same_tag_cnt);
    }
    if(same_tag_cnt>0) {
      _tag_data_prev[0] = _new_tagStr.bytes[0] = can_rx.data[4];
      _tag_data_prev[1] = _new_tagStr.bytes[1] = can_rx.data[5];
      _tag_data_prev[2] = _new_tagStr.bytes[2] = can_rx.data[6];
      _tag_data_prev[3] = _new_tagStr.bytes[3] = can_rx.data[7];
      Serial.print("New tag:");
      Serial.print(_new_tagStr.bytes[0]);Serial.print(",");
      Serial.print(_new_tagStr.bytes[1]);Serial.print(",");
      Serial.print(_new_tagStr.bytes[2]);Serial.print(",");
      Serial.print(_new_tagStr.bytes[3]);Serial.println();
      _new_tagStr.type = (TAG_Type)_new_tagStr.bytes[3];
      _cbTagEvent(_new_tagStr);
    } else {
      _same_tag_reset_timer = 2500;
    }
    break;
  case 2:   //No line
    _isLineOut = true;
    _lineOut_timer = millis() - _lineDetect_millis_last;
    if(_lineOut_timeOut_ms > 0) {
      if(_lineOut_timer > _lineOut_timeOut_ms) {
        _go_flag = false;     //Stop the line tracer
      }
    }
    _cbDataEvent(R1MSG_LINEOUT);
    break;
  }
}
/**
* @brief Process odoMsg from can bus
*/
void OMOROBOT_R1::new_can_odo(struct can_frame can_rx)
{
    _odo_r = (can_rx.data[1]|(can_rx.data[2]<<8));
    _odo_l = (can_rx.data[3]|(can_rx.data[4]<<8));
}
void OMOROBOT_R1::control_motor_VW(int V, int W)
{
    _canTxMsg_Motor.data[0] = CAN_MOTOR_CMD_VW;
    V = V*_v_dir;
    W = W*_w_dir;
    _canTxMsg_Motor.data[1] = V&0xFF;
    _canTxMsg_Motor.data[2] = (V>>8)&0xFF;
    _canTxMsg_Motor.data[3] = W&0xFF;
    _canTxMsg_Motor.data[4] = (W>>8)&0xFF;
    _mcp2515->sendMessage(&_canTxMsg_Motor);
}
void OMOROBOT_R1::request_odo()
{
  if(_odo_reset) {
    _canTxMsg_Odo.data[0] = CAN_MOTOR_ODO_RESET;
  }else {
    _canTxMsg_Odo.data[0] = CAN_MOTOR_ODO_REQUEST;
  }
  _canTxMsg_Odo.data[1] = 0;
  _canTxMsg_Odo.data[2] = 0;
  int ret = _mcp2515->sendMessage(&_canTxMsg_Odo);
  if(ret > 0) {
#ifdef DEBUG_DRIVER    
    Serial.print("TX Failed");
    Serial.println(ret);
#endif
  }
}
void OMOROBOT_R1::set_driveMode(R1_DriveMode mode)
{
  _drive_mode = mode;
  if(_drive_mode == R1DRV_LineTracerMode) {
    Serial.println("Set Line tracer mode");
    _line_error_accum = 0;
    _goal_V = 0;
    _goal_W = 0;
    _cmd_speed = 0;
    _3ms_loop = &speed_control;
    _10ms_loop = &line_control;
    _3ms_loop_millis_last = millis();
    _10ms_loop_millis_last = millis();
  }
}
void OMOROBOT_R1::set_drive_direction(Drive_DirectionType dir, Line_AlignmentType line)
{
  if(dir == Drive_Forward) {
    _v_dir = 1;
  }else if(dir == Drive_Reverse) {
    _v_dir = -1;
  }
  if(line == Line_Forward) {
    _w_dir = 1;
  }else if(line == Line_Reverse) {
    _w_dir = -1;
  }
}
void OMOROBOT_R1::set_lineoutTime(int ms)
{
  _lineOut_timeOut_ms = ms;
}
/// Start vehicle with target speed
void OMOROBOT_R1::go(int target_speed)
{
  if(target_speed) {
    _target_speed = target_speed;
    _resume_speed = target_speed;
  }
  _go_flag = true;
}
/// Resume vehicle motion when paused. 
/// If vehicle is stopped, calling this wouldn't start the vehicle
void OMOROBOT_R1::go(void)
{
  _target_speed = _resume_speed;
  _go_flag = true;
}
/// Clear go flag and stop the vehicle
void OMOROBOT_R1::stop()
{
  _target_speed = 0;
  _go_flag = false;
}
/// Only reset target speed to 0 and wait for go()
void OMOROBOT_R1::pause()
{
  _target_speed = 0;
}

bool OMOROBOT_R1::is_going()
{
  return _go_flag;
}

int OMOROBOT_R1::get_odo_l()
{
  return _odo_l;
}

int OMOROBOT_R1::get_odo_r()
{
  return _odo_r;
}

int8_t OMOROBOT_R1::get_linePos()
{
  return _line_pos;
}

int OMOROBOT_R1::get_lineoutTimer()
{
  return _lineOut_timer;
}
/// Initiate turn process to start with direction and turn odometry count from wheel
/// Turn angle is determined by odometry count and dependent to wheel size
void OMOROBOT_R1::start_turn(Turn_DirectionType dir, int turn_odo_cnt)
{
  if(dir == Turn_Right) {
    _turn_cmd = 0;
  } else if(dir == Turn_Left) {
    _turn_cmd = 1;
  }
  _turn_odo_cnt = turn_odo_cnt;
}

int OMOROBOT_R1::can_TxMsg_init(can_frame* frame, int id, int dlc)
{
    if(id>255) return 1;
    if(dlc>8 || dlc<1) return 2;
    frame->can_id = (1<<4)|id;
    frame->can_dlc = dlc;
    for(int i = 0; i<dlc; i++) {
        frame->data[i] = 0;
    }
//#ifdef DEBUG_DRIVER    
    Serial.print("Init: ID= 0x");
    Serial.print(frame->can_id, HEX);
    Serial.print(" DLC= ");
    Serial.print(frame->can_dlc);
    Serial.println();
//#endif    
    return 0;
}

