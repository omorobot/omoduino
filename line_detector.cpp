#include <Arduino.h>
#include "line_detector.h"

LINE_DETECTOR::LINE_DETECTOR()
{
   this->_line_pos = 0.0;
   this->_line_pos_last = 0.0;
   this->_lineOut_timeOut_ms = DEFAULT_LINE_DETECTOR_TIMEOUT_MS;
}

void LINE_DETECTOR::set_lineout_timeout_ms(int timeout)
{
   this->_lineOut_timeOut_ms = timeout;
}

double LINE_DETECTOR::get_line_pos(void)
{
   return this->_line_pos;
}
/**
 * @brief Try to find center position of the magnetic line from 16 bit binary data 
 * Example: [n/a][0][0][0][1][1][1][0] [0][0][0][0][0][0][0][0]
 * Add set position: 3+4+5 = 12
 * Divide by 3 = 4
 * Substract center position 4 - 7.5 = -3.5
 * @note First bit or bit[0] is ignored
 **/
LINE_DETECT_RESULT LINE_DETECTOR::detect_linePos_from_u16data(uint16_t data)
{
   LINE_DETECT_RESULT result = LINE_OK;
   double line_pos = 0.0;
   uint8_t  detectArr[15];
   int      setCount = 0;         //Stores consecutive number set in magnetic data
   // Scan line data from 1 to 16 
   for(int i = 1; i<16; i++) {
      if((data>>i)&0x01) {
         detectArr[setCount] = i-1;
         setCount++;
      }
   }
   double gain = 1.3;
   if( setCount>0 ) {      //Atleast 1 candidate found in the sensor
      for(int j = 0; j<setCount; j++) {
         line_pos+=detectArr[j];
      }
      line_pos = line_pos / setCount - 7;
      line_pos = line_pos * gain;
      _line_pos_last = line_pos;       //Remember last known line pos 
      _is_line_out = false;
      _lineDetect_millis_last = millis();    //Update line detection time
      _lineOut_timer = 0;
   } 
   else {                            //No line is found
      _is_line_out = true;
      result = LINE_OUT;
      line_pos = _line_pos_last*1.3;   //Set line position as last known line pos
      if( (millis() - _lineDetect_millis_last) > _lineOut_timeOut_ms) {    //_lineOut_timeOut_ms is not working
         result = LINE_OUT_TIMEOUT;
      }
   }
   return result;
}