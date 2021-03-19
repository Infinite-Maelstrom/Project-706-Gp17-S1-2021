#include <math.h>

enum SIDE {left, right};

int front_mm;
int rear_mm;
double thetad;
int thetai;
int deg;
int dist;

int rad2deg(double rad){
  deg = (int)(rad * 57296 / 1000);
  return deg;
}

void adjust_readings(){
  //add the distance of each sensor to the centreline
  front_mm += 35;
  rear_mm += 75;
}

int dist2centre(int front_mm, int rear_mm){
  //take weighted average
  //numbers are distances in tenths of mm to avoid type casting
  dist = (front_mm * 5715 + rear_mm * 8255)/(5715 + 8255);
  return dist;  
}

int angle(int front_mm, int rear_mm, enum SIDE side){
  //use arctan.  
  if (side == 1) {
    thetad = atan(((front_mm - rear_mm)* 10)/1397);
  } else {
    thetad = atan(((front_mm - rear_mm)* 10)/1397);
    thetad *= -1;
  }
  thetai = rad2deg(thetad);
  return thetai;
}
