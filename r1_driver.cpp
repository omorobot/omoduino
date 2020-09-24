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
}
OMOROBOT_R1::OMOROBOT_R1() {
  _mcp2515 = new MCP2515(10);
  _drive_mode = R1DRV_None;
}

OMOROBOT_R1::OMOROBOT_R1(uint16_t cspin) {
  _mcp2515 = new MCP2515(cspin);
  _drive_mode = R1DRV_None;
}

void OMOROBOT_R1::begin() {
  SPI.begin();
  _mcp2515->reset();
  _mcp2515->setBitrate(CAN_500KBPS);
  _mcp2515->setNormalMode();
  can_TxMsg_init(&_canTxMsg_Motor, 0x4, 8);
  can_TxMsg_init(&_canTxMsg_Odo, 0x4, 8);
  _odoRequest_millis_last = millis();
}

void OMOROBOT_R1::onNewData(R1_NewDataClientEvent cbEvent)
{
  _cbEvent = cbEvent;
}
void OMOROBOT_R1::spin() {
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
          _cbEvent(R1MSG_LINEPOS);
          break;
        case 2:   //No line
          _isLineOut = true;
          _lineOut_timer = millis() - _lineDetect_millis_last;
          if(_lineOut_timeOut_ms > 0) {
            if(_lineOut_timer > _lineOut_timeOut_ms) {
              _go_flag = false;     //Stop the line tracer
            }
          }
          
          _cbEvent(R1MSG_LINEOUT);
          break;
      }
    } else if(senderID == 0x4) {
      if(_canRxMsg.data[0] == 0x02) {
        _odo_r = (_canRxMsg.data[1]|(_canRxMsg.data[2]<<8));
        _odo_l = (_canRxMsg.data[3]|(_canRxMsg.data[4]<<8));
        _cbEvent(R1MSG_ODO);
      }
#ifdef DEBUG_DRIVER
      Serial.print("ODO: L= ");
      Serial.print(_odo_l);
      Serial.print(" R= ");
      Serial.print(_odo_r);
      Serial.println();
#endif
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
}
void OMOROBOT_R1::control_motor_VW(int V, int W)
{
    _canTxMsg_Motor.data[0] = CAN_MOTOR_CMD_VW;
    _canTxMsg_Motor.data[1] = V&0xFF;
    _canTxMsg_Motor.data[2] = (V>>8)&0xFF;
    _canTxMsg_Motor.data[3] = W&0xFF;
    _canTxMsg_Motor.data[4] = (W>>8)&0xFF;
    _mcp2515->sendMessage(&_canTxMsg_Motor);
}
void OMOROBOT_R1::request_odo()
{
  if(_odoReset) {
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
  if(_drive_mode == R1DRV_LineTracer) {
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
void OMOROBOT_R1::set_lineoutTime(int ms)
{
  _lineOut_timeOut_ms = ms;
}
void OMOROBOT_R1::go(int target_speed)
{
  if(target_speed) {
    _target_speed = target_speed;
    _resume_speed = target_speed;
  }
  _go_flag = true;
}
void OMOROBOT_R1::go(void)
{
  _target_speed = _resume_speed;
  _go_flag = true;
}
void OMOROBOT_R1::stop()
{
  _target_speed = 0;
  _go_flag = false;
}
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
int OMOROBOT_R1::can_TxMsg_init(can_frame* frame, int id, int dlc)
{
    if(id>255) return 1;
    if(dlc>8 || dlc<1) return 2;
    frame->can_id = (1<<4)|id;
    frame->can_dlc = dlc;
    for(int i = 0; i<dlc; i++) {
        frame->data[i] = 0;
    }
#ifdef DEBUG_DRIVER    
    Serial.print("Init: ID= 0x");
    Serial.print(frame->can_id, HEX);
    Serial.print(" DLC= ");
    Serial.print(frame->can_dlc);
    Serial.println();
#endif    
    return 0;
}

