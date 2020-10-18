#include "r1_command.h"
#include <mcp2515.h>

R1_CanBus::R1_CanBus(void)
{
   _mcp2515 = new MCP2515(10);
   can_TxMsg_init(&_can_tx_cmd, 0x4, 8);
   can_TxMsg_init(&_can_tx_odo, 0x4, 8);
}
R1_CanBus::R1_CanBus(uint16_t cspin)
{
   _mcp2515 = new MCP2515(cspin);
   can_TxMsg_init(&_can_tx_cmd, 0x4, 8);
   can_TxMsg_init(&_can_tx_odo, 0x4, 8);
}
R1_CanBus::R1_CanBus(MCP2515* mcp2515)
{
   _mcp2515 = mcp2515;
   can_TxMsg_init(&_can_tx_cmd, 0x4, 8);
   can_TxMsg_init(&_can_tx_odo, 0x4, 8);
}
void R1_CanBus::begin_bus(void)
{
   SPI.begin();
   _mcp2515->reset();
   _mcp2515->setBitrate(CAN_500KBPS);
   _mcp2515->setNormalMode();
   Serial.println("R1 command bus begin");
}
void R1_CanBus::set_vehicle_type(R1_VEHICLE_TYPE type) {
   _vehicle_type = type;
   set_control_mode(CONTROL_MODE_VW);
}
void R1_CanBus::set_control_mode(R1_CONTROL_MODE_TYPE mode)
{
   switch(mode) {
   case CONTROL_MODE_VW:
      if(_vehicle_type == VEHICLE_TYPE_PL153) {
         _canCmd.cmd_byte = CAN_MOTOR_CMD_VW_PL;
      } else {
         _canCmd.cmd_byte = CAN_MOTOR_CMD_VW;
      }
      break;
   case CONTROL_MODE_DIFFV:
      if(_vehicle_type!=VEHICLE_TYPE_PL153) {
         _canCmd.cmd_byte = CAN_MOTOR_CMD_DIFF_V;
      }
      break;
   case CONTROL_MODE_RPM:
      if(_vehicle_type!=VEHICLE_TYPE_PL153) {
         _canCmd.cmd_byte = CAN_MOTOR_CMD_RPM;
      }
      break;
   case CONTROL_MODE_DAC_ANGLE:
      if(_vehicle_type == VEHICLE_TYPE_PL153) {
         _canCmd.cmd_byte = CAN_MOTOR_CMD_DAC_ANGLE;
      }
      break;
   }   
}
void R1_CanBus::onNewCanRx(OMOROBOT_R1* obj, R1_NewCanRxEvent cbEvent)
{
   _cbObj = obj;
   _cbCanRxEvent = cbEvent;
}
void R1_CanBus::scan(void)
{
   if(_mcp2515->readMessage(&_canRxMsg) == MCP2515::ERROR_OK) {
      (this->_cbObj->*_cbCanRxEvent)(_canRxMsg);
   }
}


void R1_CanBus::set_pl_lift_mode(PL_LIFT_MODE_TYPE mode)
{
   switch(mode) {
      case PL_LIFT_STOP:
      _canCmd.aux_byte = 0;
      break;
      case PL_LIFT_UP:
      _canCmd.aux_byte = 1;
      break;
      case PL_LIFT_DOWN:
      _canCmd.aux_byte = 2;
      break;
   }
}
void R1_CanBus::request_odo(bool reset)
{
   if(reset) {
      _can_tx_odo.data[0] = CAN_MOTOR_ODO_RESET;
   }else {
      _can_tx_odo.data[0] = CAN_MOTOR_ODO_REQUEST;
   }
   _can_tx_odo.data[1] = 0;
   _can_tx_odo.data[2] = 0;
   int ret = _mcp2515->sendMessage(&_can_tx_odo);
   if(ret > 0) {
   #ifdef DEBUG_DRIVER    
      Serial.print("TX Failed");
      Serial.println(ret);
   #endif
   }
}
void R1_CanBus::cmd_VW(int16_t v_mm_s, int16_t w_mrad_s)
{
   if(_control_mode!=CONTROL_MODE_VW) {
      set_control_mode(CONTROL_MODE_VW);
   }
   _canCmd.data_1 = v_mm_s;
   _canCmd.data_2 = w_mrad_s;
   sendCommand(_canCmd);
}
void R1_CanBus::cmd_diffv(int16_t v_l_mm_s, int16_t v_r_mm_s)
{
   if(_control_mode!=CONTROL_MODE_DIFFV) {
      set_control_mode(CONTROL_MODE_DIFFV);
   }
   _canCmd.data_1 = v_l_mm_s;
   _canCmd.data_2 = v_r_mm_s;
   sendCommand(_canCmd);
}
void R1_CanBus::cmd_pl_dac_angle(int16_t dac, int16_t angle)
{
   if(_control_mode!=CONTROL_MODE_DAC_ANGLE) {
      set_control_mode(CONTROL_MODE_DAC_ANGLE);
   }
   _canCmd.data_1 = dac;
   _canCmd.data_2 = angle;
   sendCommand(_canCmd);
}
void R1_CanBus::sendCommand(CanCommandType cmd)
{
   _can_tx_cmd.data[0] = cmd.cmd_byte;
   _can_tx_cmd.data[1] = cmd.data_1&0xFF;
   _can_tx_cmd.data[2] = (cmd.data_1>>8)&0xFF;
   _can_tx_cmd.data[3] = cmd.data_2&0xFF;
   _can_tx_cmd.data[4] = (cmd.data_2>>8)&0xFF;
   _can_tx_cmd.data[5] = cmd.aux_byte;
   _mcp2515->sendMessage(&_can_tx_cmd);

}
int R1_CanBus::can_TxMsg_init(can_frame* frame, int id, int dlc)
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