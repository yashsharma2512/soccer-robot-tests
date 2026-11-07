#pragma once
#include <stdint.h>
#include <string.h>
namespace benchlogic {
inline bool number(const char *text,int minimum,int maximum,int &result) {
  if (!text || !*text) return false;
  int value=0;
  for (unsigned i=0;text[i];i++) {
    if (i>=5 || text[i]<'0' || text[i]>'9') return false;
    value=value*10+(text[i]-'0');
    if (value>maximum) return false;
  }
  if (value<minimum) return false;
  result=value; return true;
}
// Logical wheel PWM before BL polarity inversion. No heading correction here.
inline bool motion(const char *name,int pwm,int16_t (&wheel)[4]) {
  if (pwm<30 || pwm>120) return false;
  int forward=0,strafe=0,rotation=0;
  if (!strcmp(name,"forward")) forward=pwm;
  else if (!strcmp(name,"back")) forward=-pwm;
  else if (!strcmp(name,"left")) strafe=-pwm;
  else if (!strcmp(name,"right")) strafe=pwm;
  else if (!strcmp(name,"rotatePlus")) rotation=pwm;
  else if (!strcmp(name,"rotateMinus")) rotation=-pwm;
  else return false;
  wheel[0]=int16_t(forward+strafe+rotation); wheel[1]=int16_t(forward-strafe-rotation);
  wheel[2]=int16_t(forward-strafe+rotation); wheel[3]=int16_t(forward+strafe-rotation);
  return true;
}
}
