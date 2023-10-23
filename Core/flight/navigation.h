#ifndef _NAVIGATION_
#define _NAVIGATION_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"

typedef struct{
  void* func;
  uint8_t priority;
  uint8_t init;
}flymode;

typedef enum{
  NULL_WP = 0,
  TAKE_OFF,
  WAYPOINT,
  LANDING
}wp_type;

typedef struct{
  int32_t lat;
  int32_t lon;
  int32_t alt;
  int8_t Type;
}waypoint;
extern uint16_t autopilot_stick;
extern uint16_t circle_stick;
extern uint16_t rtHome_stick;
void Autopilot();

#ifdef __cplusplus
}
#endif
#endif