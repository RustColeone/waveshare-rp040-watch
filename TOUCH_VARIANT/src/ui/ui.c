// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.3
// LVGL version: 8.2.0
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1( lv_event_t * e);
lv_obj_t *ui_Screen1;
lv_obj_t *ui_ImageWatchface;
lv_obj_t *ui_BatteryPercentageArc;
lv_obj_t *ui_BatteryVoltageLabel;
lv_obj_t *ui_DigitalTimeLabel;
lv_obj_t *ui_ImageArmHour;
lv_obj_t *ui_ImageArmMinute;
lv_obj_t *ui_ImageArmSecond;


// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
void ui_event_Screen2( lv_event_t * e);
lv_obj_t *ui_Screen2;
lv_obj_t *ui_BrightnessArc;
lv_obj_t *ui_Label3;


// SCREEN: ui_Screen3
void ui_Screen3_screen_init(void);
void ui_event_Screen3( lv_event_t * e);
lv_obj_t *ui_Screen3;
lv_obj_t *ui_Roller1;
lv_obj_t *ui_Roller2;
lv_obj_t *ui_Roller3;
lv_obj_t *ui_Roller4;
lv_obj_t *ui_StartTimerButton;
lv_obj_t *ui_Label1;
lv_obj_t *ui_Label7;


// SCREEN: ui_Screen4
void ui_Screen4_screen_init(void);
lv_obj_t *ui_Screen4;
lv_obj_t *ui_TimerArc;
lv_obj_t *ui_EndTimerButton;
lv_obj_t *ui_Label2;


// SCREEN: ui_Screen5
void ui_Screen5_screen_init(void);
void ui_event_Screen5( lv_event_t * e);
lv_obj_t *ui_Screen5;
lv_obj_t *ui_DataArc1;
lv_obj_t *ui_DataArc2;
lv_obj_t *ui_DataArc3;


// SCREEN: ui_Screen6
void ui_Screen6_screen_init(void);
void ui_event_Screen6( lv_event_t * e);
lv_obj_t *ui_Screen6;
lv_obj_t *ui_Roller5;
lv_obj_t *ui_Roller6;
lv_obj_t *ui_Roller7;
lv_obj_t *ui_Roller8;
lv_obj_t *ui_SetTimeButton;
lv_obj_t *ui_Label6;
lv_obj_t *ui____initial_actions0;
const lv_img_dsc_t *ui_imgset_out[8] = {&ui_img_out1_png, &ui_img_out2_png, &ui_img_out3_png, &ui_img_out4_png, &ui_img_out5_png, &ui_img_out6_png, &ui_img_out7_png, &ui_img_out8_png};
const lv_img_dsc_t *ui_imgset_watchface[1] = {&ui_img_watchface240_png};

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Screen1( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( &ui_Screen3, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, &ui_Screen3_screen_init);
}
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( &ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, &ui_Screen2_screen_init);
}
if ( event_code == LV_EVENT_LONG_PRESSED) {
      _ui_screen_change( &ui_Screen6, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, &ui_Screen6_screen_init);
}
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_BOTTOM  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( &ui_Screen5, LV_SCR_LOAD_ANIM_MOVE_BOTTOM, 250, 0, &ui_Screen5_screen_init);
}
}
void ui_event_Screen2( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_LEFT  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_LEFT, 250, 0, &ui_Screen1_screen_init);
}
}
void ui_event_Screen3( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_RIGHT  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 250, 0, &ui_Screen1_screen_init);
}
}
void ui_event_Screen5( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_GESTURE &&  lv_indev_get_gesture_dir(lv_indev_get_act()) == LV_DIR_TOP  ) {
lv_indev_wait_release(lv_indev_get_act());
      _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_MOVE_TOP, 250, 0, &ui_Screen1_screen_init);
}
}
void ui_event_Screen6( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_LONG_PRESSED) {
      _ui_screen_change( &ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 250, 0, &ui_Screen1_screen_init);
}
}

///////////////////// SCREENS ////////////////////

void ui_init( void )
{
lv_disp_t *dispp = lv_disp_get_default();
lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
lv_disp_set_theme(dispp, theme);
ui_Screen1_screen_init();
ui_Screen2_screen_init();
ui_Screen3_screen_init();
ui_Screen4_screen_init();
ui_Screen5_screen_init();
ui_Screen6_screen_init();
ui____initial_actions0 = lv_obj_create(NULL);
lv_disp_load_scr( ui_Screen1);
}
