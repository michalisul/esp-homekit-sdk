
#ifndef app_sw_h_
#define app_sw_h_

#include <stdbool.h>

#define RELAY_PIN 5

void init_hw(void);
void switch_set(bool);
bool switch_get(void);

#endif