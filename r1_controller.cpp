#include <Arduino.h>
#include "r1_controller.h"

/**
 * @brief Construct a new r1 controller::r1 controller object
 * 
 */
R1_Controller::R1_Controller()
{
   _pid_l.Kp =             PID_LINE_KP_DEFAULT;
   _pid_l.Ki =             PID_LINE_KI_DEFAULT;
   _pid_l.Kd =             PID_LINE_KD_DEFAULT;
   _pid_l.error_i_max =    PID_LINE_ERROR_I_MAX_DEFAULT;
   _pid_l.out_max =        PID_LINE_OUT_MAX_DEFAULT;
   _pid_l.error_i =        0.0;   
   _line_filter_alpha =    PID_LINE_FILTER_ALPHA_DEFAULT;
   _v_accel =              V_CONTROL_ACCEL_DEFAULT;
}
/**
 * @brief Initialize PID gain setting for line tracer type vehicle
 * 
 * @param pid PID gains
 */
void  R1_Controller::set_pid_gain_line(PID_Type pid)
{
   _pid_l.Kp = pid.Kp;
   _pid_l.Ki = pid.Ki;
   _pid_l.Kd = pid.Kd;
   _pid_l.out_max = pid.out_max;
   _pid_l.error_i_max = pid.error_i_max;
   _pid_l.error_prev = 0.0;
   _pid_l.error_i = 0.0;
}
/**
 * @brief Reset integral error
 * 
 */
void  R1_Controller::reset_pid_line(void)
{
   _pid_l.error_i = 0.0;
}
/**
 * @brief Set target speed
 * 
 * @param v 
 */
void  R1_Controller::set_target_v(int v)
{
   _v_target = v;
}
/**
 * @brief Set target accelleration
 * 
 * @param accel 
 */
void  R1_Controller::set_v_accel(int accel)
{
   _v_accel = accel;
}
/**
 * @brief Speed control
 * 
 * @param cmd_v target speed V mm/s
 * @param go_flag true to set go vehicle
 * @return int commandded v mm/s
 */
int   R1_Controller::speed_control(int cmd_v, bool go_flag)
{
   if(go_flag) {
      if(cmd_v < _v_target) {
         cmd_v+=_v_accel;
      } else if(cmd_v > _v_target) {
         cmd_v-=_v_accel;   //Decelerate faster
      }
   } else {
      if(cmd_v > 1) {
         cmd_v-=_v_accel;
      } else if(cmd_v < -1) {
         cmd_v+=_v_accel;
      } else {
         cmd_v = 0;
      }
   }
   return cmd_v;
}
/**
 * @brief 
 * 
 * @param origin_val 
 * @param target_val 
 * @param increase 
 * @return int 
 */
int   R1_Controller::speed_w_control(int origin_val, int target_val, int increase)
{
   if(target_val > origin_val){
      if(target_val > (origin_val + increase)){
         return origin_val + increase;
      } else{
         return target_val;
      }
   } else{
      if(target_val < (origin_val - increase)){
         return origin_val - increase;
      }else{
         return target_val;
      }
   }

}
/**
 * @brief Compute desired W mrad/s with line position as reference
 * 
 * @param linePos Detected line position from -10.0 to +10.0
 * @return int Desired W mrad/s
 */
int   R1_Controller::line_control_vw(double linePos)
{
   double dt = (double)(millis()-_pid_l.last_update_millis)/1000.0;
   _pid_l.last_update_millis = millis();
   double error = linePos*_line_filter_alpha + (1.0-_line_filter_alpha)*_pid_l.error_prev;
   _pid_l.error_i += error * dt;
   double error_d = (error - _pid_l.error_prev)/dt;
   _pid_l.error_prev = error;  
   if(_pid_l.error_i > _pid_l.error_i_max) _pid_l.error_i = _pid_l.error_i_max;
   else if(_pid_l.error_i < -_pid_l.error_i_max) _pid_l.error_i = -_pid_l.error_i_max;
   double output = _pid_l.Kp * error + _pid_l.error_i*_pid_l.Ki + error_d * _pid_l.Kd;
   if(output > _pid_l.out_max) output = _pid_l.out_max;
   else if(output < -_pid_l.out_max) output = -_pid_l.out_max;

   return (int)output;
}
/**
 * @brief Compute desired steering angle with line position as reference
 * 
 * @param linePos Detected line position from -10.0 to +10.0
 * @return int Desired steering angle
 */
int   R1_Controller::line_control_angle(double linePos)
{
   double error = linePos*_line_filter_alpha + (1.0-_line_filter_alpha)*_pid_l.error_prev;
   _pid_l.error_i += error;
   double error_d = error - _pid_l.error_prev;
   _pid_l.error_prev = error;  
   if(_pid_l.error_i > _pid_l.error_i_max) _pid_l.error_i = _pid_l.error_i_max;
   else if(_pid_l.error_i < -_pid_l.error_i_max) _pid_l.error_i = -_pid_l.error_i_max;
   double output = _pid_l.Kp * error + _pid_l.error_i*_pid_l.Ki + error_d * _pid_l.Kd;
   if(output > _pid_l.out_max) output = _pid_l.out_max;
   else if(output < -_pid_l.out_max) output = -_pid_l.out_max;
   return (int)output;
}
