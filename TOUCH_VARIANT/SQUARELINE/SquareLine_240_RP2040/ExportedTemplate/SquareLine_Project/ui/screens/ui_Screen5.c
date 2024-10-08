// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.3
// LVGL version: 8.2.0
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Screen5_screen_init(void)
{
ui_Screen5 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen5, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_DataArc1 = lv_arc_create(ui_Screen5);
lv_obj_set_width( ui_DataArc1, 240);
lv_obj_set_height( ui_DataArc1, 240);
lv_obj_set_align( ui_DataArc1, LV_ALIGN_CENTER );
lv_arc_set_value(ui_DataArc1, 100);
lv_arc_set_bg_angles(ui_DataArc1,180,90);

lv_obj_set_style_arc_color(ui_DataArc1, lv_color_hex(0xFF4040), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_DataArc1, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_DataArc1, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_DataArc1, 0, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_DataArc2 = lv_arc_create(ui_Screen5);
lv_obj_set_width( ui_DataArc2, 180);
lv_obj_set_height( ui_DataArc2, 180);
lv_obj_set_align( ui_DataArc2, LV_ALIGN_CENTER );
lv_arc_set_value(ui_DataArc2, 100);
lv_arc_set_bg_angles(ui_DataArc2,180,90);

lv_obj_set_style_arc_color(ui_DataArc2, lv_color_hex(0xFFA140), LV_PART_INDICATOR | LV_STATE_DEFAULT );
lv_obj_set_style_arc_opa(ui_DataArc2, 255, LV_PART_INDICATOR| LV_STATE_DEFAULT);

lv_obj_set_style_bg_color(ui_DataArc2, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_DataArc2, 0, LV_PART_KNOB| LV_STATE_DEFAULT);

ui_DataArc3 = lv_arc_create(ui_Screen5);
lv_obj_set_width( ui_DataArc3, 120);
lv_obj_set_height( ui_DataArc3, 120);
lv_obj_set_align( ui_DataArc3, LV_ALIGN_CENTER );
lv_arc_set_value(ui_DataArc3, 100);
lv_arc_set_bg_angles(ui_DataArc3,180,90);

lv_obj_set_style_bg_color(ui_DataArc3, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_DataArc3, 0, LV_PART_KNOB| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_Screen5, ui_event_Screen5, LV_EVENT_ALL, NULL);

}
