#include <Arduino.h>
#include "src/omoduino/r1_driver.h"
#include "src/omoduino/sonar.h"

OMOROBOT_R1 r1;

const int PIN_TRIGGER1 = 2;
const int PIN_ECHO1 = 3;
const int PIN_TRIGGER2 = 4;
const int PIN_ECHO2 = 5;

SONAR sonar_L = SONAR(PIN_TRIGGER1, PIN_ECHO1);
SONAR sonar_R = SONAR(PIN_TRIGGER2, PIN_ECHO2);

const int PIN_GO = 6;
const int PIN_STOP = 7;


uint64_t  sonar_update_millis_last = millis();
double    sonar_distance_L = 0.0;
double    sonar_distance_R = 0.0;
int sonar_read_state = 0;

void loop_update_sonar()
{
   // Read left and right sonar measurement
   if(sonar_read_state++%2) {
      sonar_distance_L = sonar_L.measure_cm();
   } else {
      sonar_distance_R = sonar_R.measure_cm();
   }
}

void newR1_message_event(R1_MessageType msgType) {
   switch (msgType) {
   case R1MSG_ODO:
      int odo_l = r1.get_odo_l();
      int odo_r = r1.get_odo_r();
      break;
   case R1MSG_LINEPOS:
      int line_pos = r1.get_linePos();
      break;
   case R1MSG_LINEOUT:
      int lineout_time = r1.get_lineoutTimer();
      break;
   default:
      break;
   }
}


void setup() {
   Serial.begin(115200);
   pinMode(PIN_GO, INPUT);
   pinMode(PIN_STOP, INPUT);
   pinMode(PIN_TRIGGER1, OUTPUT);
   pinMode(PIN_ECHO1, INPUT);
   pinMode(PIN_TRIGGER2, OUTPUT);
   pinMode(PIN_ECHO2, INPUT);
   // Set detection range for sonar
   sonar_L.set_range(60.0);
   sonar_R.set_range(60.0);
   // Set drive mode to Line tracer
   r1.set_driveMode(R1DRV_LineTracer);
   // Set timeout to stop vehicle when line out detected : 2000ms
   r1.set_lineoutTime(2000);
   // Assign callback function for new message
   r1.onNewData(newR1_message_event);
   // Start running r1
   r1.begin();
}

void loop() {
   if(millis() - sonar_update_millis_last > 19) {    //For every 20ms
      loop_update_sonar();
      //Check if anything detected in the sensor
      if(sonar_L.detected() || sonar_R.detected()) {
         r1.pause();
      } else {
         //Robot is cleared
         if(r1.is_going()) {
         r1.go();
         }
      }
      sonar_update_millis_last = millis();
   }
   if(digitalRead(PIN_GO)) {
      r1.go(250);
   }
   if(digitalRead(PIN_STOP)) {
      r1.stop();
   }
   r1.spin();
}
