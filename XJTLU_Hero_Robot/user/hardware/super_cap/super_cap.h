#ifndef SUPER_CAP_H
#define SUPER_CAP_H
#include "main.h"

#define SUPER_CAP_CTRL_ON_KEY KEY_PRESSED_OFFSET_V
#define SUPER_CAP_CTRL_OFF_KEY KEY_PRESSED_OFFSET_B
#define SUPER_CAP_AUTO_CHARGING_POWER		40.0f

typedef enum
{
		SUPER_CAP_ON,
		SUPER_CAP_OFF,
} super_cap_state_t;

extern void super_cap_configuration(void);
extern void super_cap_on(void);
extern void super_cap_off(void);

#endif
