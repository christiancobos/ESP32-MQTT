/*
 * lvgl_demo_ui.h
 *
 *  Created on: 17 nov 2023
 *      Author: jcgar
 */

#ifndef MAIN_LVGL_DEMO_UI_H_
#define MAIN_LVGL_DEMO_UI_H_

#include "lvgl.h"

extern void ui_update_clock(uint8_t hout, uint8_t minute, uint8_t second );
extern void ui_example_lvgl_demo_init(lv_disp_t *disp);
extern void ui_set_indicator_value(float value);

// Control de LEDs
extern void ui_set_red_led_on(void);
extern void ui_set_red_led_off(void);
extern void ui_toggle_blue_led(void);
extern void ui_set_green_led_on(void);
extern void ui_set_green_led_off(void);


#endif /* MAIN_LVGL_DEMO_UI_H_ */
