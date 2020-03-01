#ifndef FRIC_H
#define FRIC_H
#include "main.h"

#define Fric_UP 1300
#define Fric_DOWN 1360
#define Fric_OFF 1450

/*
·¶Î§1000-2000
mid 1450  
speed 28m/s 1360

*/

extern void fric_PWM_configuration(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
