#ifndef FRIC_H
#define FRIC_H
#include "main.h"

#define Fric_UP 1220
#define Fric_DOWN 1220
#define Fric_OFF 1000

#define Fric_Big_UP 1800
#define Fric_Big_DOWN 1550		//1800 一到两颗超射速		//转速较小1300
#define Fric_Big_OFF 1000

/*
范围1000-2000
mid 1450  
speed 28m/s 1360

*/

extern void fric_PWM_configuration(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);

extern void fric_Big_PWM_configuration(void);
extern void fric_big_off(void);
extern void fric1_big_on(uint16_t cmd);
extern void fric2_big_on(uint16_t cmd);
#endif
