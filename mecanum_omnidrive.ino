/*MECANUM OMNIDRIVE
 * This file contains various functions for moving a mecanum-wheeled robot,
 * specific quirks for the robot used in MECHENG 706 are included, as that
 * is the primary target hardware for this code.
 * 
 * Note: as of version 2.0.0, all functions except void motion(<args>) are 
 * obsolete.  Please do not use the other functions, they are very buggy.
 */

 /*
  * Author: 
  *   James Donaldson, jdon759, 763530765.
  * Version: 2.0.1
  * Date of latest version:
  *   2021-03-17
  */
#include <Servo.h>

//Please note that no control logic is provided in these functions

enum ROTATION {
  CW = -1,
  NONE = 0,
  CCW = 1
};

Servo left_front;
Servo left_rear;
Servo right_front;
Servo right_rear;

int error;
uint8_t count = 0;
int val = 1;
/*int rot_dir;
int heading;
int pwr;*/
int strafe;
int forward;

void setup() {
  // put your setup code here, to run once:
  // Attach servos to the correct outputs
  left_front.attach(46);
  left_rear.attach(47);
  right_front.attach(50);
  right_rear.attach(51);

}

/*=====================main loop======================*/
void loop() {
  // put your main code here, to run repeatedly:
  
  /*motion(400, 30, CCW, 40.0);
  delay(5000);
  for (int i = 0; i < 5; i++){
    motion((i*100), (i*25), NONE, 0.0);
    delay(1000);
  }
  motion(200, 270, CW, 30.0);
  delay(5000);*/
  //motion(0, 0, NONE, 0.0);
  //delay(2000);
  delay(5000);
  motion(100, 0, NONE, 0.0);
  
 
  //if (error) return;
}
/*====================================================*/


//try with ROTATION enum??
int rotate(int rot_dir, int pwr){
  // for rotating on the spot
  // pwr aceppts a number between 0 and 500, will error if else.
  if (pwr >= 0 && pwr <= 500) {
    pwr = pwr * rot_dir;
    left_front.writeMicroseconds(500 - pwr);
    left_rear.writeMicroseconds(500 - pwr);
    right_front.writeMicroseconds(500 + pwr);
    right_rear.writeMicroseconds(500 + pwr);
  }
  else if (pwr < 0 || pwr > 500){
    //return the value out of range code
    return 2;
  } else {
    return 1;
  }
  
  return 0;
}

int translate (int heading, int pwr) {
  //for motion witout rotation
  if (pwr >= 0 && pwr <= 500) {
    //calculate motion
    forward = (int)(pwr * cos((heading * 180)/PI));
    strafe = (int)(pwr * sin((heading * 180)/PI));
    // command motion
    left_front.writeMicroseconds(500 + forward + strafe);
    left_rear.writeMicroseconds(500 + forward - strafe);
    right_front.writeMicroseconds(500 + forward - strafe);
    right_rear.writeMicroseconds(500 + forward + strafe);
  } else if (pwr < 0 || pwr > 500) {
    return 2;
  } else {
    return 1;
  }
  return 0;
}

// motion() moves the robot in almsot any way you wish, including rotation.
// Do note that this is only an open loop function.  It is reccomended to 
// incorporate this function into a closed loop system for best performance.
void motion(int power, int heading, ROTATION rot_direction, float rot_percent){
  //Moves wheels of robot in roughly the correct proportions to achieve
  // holonomic motion, EXCEPT for rotation while driving straight.
  //  INPUTS:
  //    power:   an integer between 0 and 500 which dictates the speed of motion
  //             NOTE:  it is reccomended to keep this value at 400 or below.
  //    heading: an integer in degrees which tells the robot which way to drive
  //    rot_direction: a value of type ROTATION.  Can be either CW, NONE, or CCW
  //    rot_percent: a value between 0 and 100 which determines the proportion 
  //                 of rotative to translative motion
  //  OUTPUTS:
  //    none

  //version 1.0
  
  float fwd;      //Forward component
  float str;      //strafe component
  float rot;      //rotation component
  float norm;     //for normalising
  float lf, lr, rf, rr; //motor power values
  
  float trans_percent;

  // determine percent of performance dedicated to each of moving and rotating
  rot_percent /= 100.0;
  trans_percent = 1 - rot_percent;
  
  //assign values to the un-normalised componenets
  fwd = (float)power * cos((heading * 180)/PI) * trans_percent;
  str = (float)power * sin((heading * 180)/PI) * trans_percent;
  rot = (float)power * rot_percent * rot_direction;

  // if components are 0, exit. (this would indicate a bug)
  if((fwd==0)&&(str==0)&&(rot==0)){return;}

  //normalise component levels
  norm = abs(fwd) + abs(str) + abs(rot);
  lf = (fwd+str-rot)/norm;
  lr = (fwd-str-rot)/norm;
  rf = (fwd-str+rot)/norm;
  rr = (fwd+str+rot)/norm;

  //turn normalised power levels into proper power levels
  lf *= power;
  lr *= power;
  rf *= power;
  rr *= power;

  //set motor power values
  left_front.writeMicroseconds(1500 + (int)lf);
  left_rear.writeMicroseconds(1500 + (int)lr);
  right_front.writeMicroseconds(1500 - (int)rf);
  right_rear.writeMicroseconds(1500 - (int)rr);

  //done ;)
  return;
}


void rotateANDdrive(int power, int heading, ROTATION rot_direction, float rot_percent){
  //this function allows the robot to drive in a straight line while rotating.
  //In practice, it is better to use motion(<args>) inside a control loop for this purpose

  //version 0.0
  return;
}
