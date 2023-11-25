// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.3
// LVGL version: 8.2.0
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Screen3_screen_init(void)
{
ui_Screen3 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Roller1 = lv_roller_create(ui_Screen3);
lv_roller_set_options( ui_Roller1, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n5\n6\n7\n8\n9", LV_ROLLER_MODE_NORMAL );
lv_obj_set_height( ui_Roller1, 100);
lv_obj_set_width( ui_Roller1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_Roller1, -75 );
lv_obj_set_y( ui_Roller1, -20 );
lv_obj_set_align( ui_Roller1, LV_ALIGN_CENTER );

ui_Roller2 = lv_roller_create(ui_Screen3);
lv_roller_set_options( ui_Roller2, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n5\n6\n7\n8\n9", LV_ROLLER_MODE_NORMAL );
lv_obj_set_height( ui_Roller2, 100);
lv_obj_set_width( ui_Roller2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_Roller2, -25 );
lv_obj_set_y( ui_Roller2, -20 );
lv_obj_set_align( ui_Roller2, LV_ALIGN_CENTER );

ui_Roller3 = lv_roller_create(ui_Screen3);
lv_roller_set_options( ui_Roller3, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n5\n6\n7\n8\n9", LV_ROLLER_MODE_NORMAL );
lv_obj_set_height( ui_Roller3, 100);
lv_obj_set_width( ui_Roller3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_Roller3, 25 );
lv_obj_set_y( ui_Roller3, -20 );
lv_obj_set_align( ui_Roller3, LV_ALIGN_CENTER );

ui_Roller4 = lv_roller_create(ui_Screen3);
lv_roller_set_options( ui_Roller4, "0\n1\n2\n3\n4\n5\n6\n7\n8\n9\n5\n6\n7\n8\n9", LV_ROLLER_MODE_NORMAL );
lv_obj_set_height( ui_Roller4, 100);
lv_obj_set_width( ui_Roller4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_x( ui_Roller4, 75 );
lv_obj_set_y( ui_Roller4, -20 );
lv_obj_set_align( ui_Roller4, LV_ALIGN_CENTER );

ui_StartTimerButton = lv_btn_create(ui_Screen3);
lv_obj_set_width( ui_StartTimerButton, 100);
lv_obj_set_height( ui_StartTimerButton, 30);
lv_obj_set_x( ui_StartTimerButton, 0 );
lv_obj_set_y( ui_StartTimerButton, 60 );
lv_obj_set_align( ui_StartTimerButton, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_StartTimerButton, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_StartTimerButton, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Label1 = lv_label_create(ui_StartTimerButton);
lv_obj_set_width( ui_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label1,"Start");

ui_Label7 = lv_label_create(ui_Screen3);
lv_obj_set_width( ui_Label7, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label7, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label7, 0 );
lv_obj_set_y( ui_Label7, -80 );
lv_obj_set_align( ui_Label7, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label7,"Timer");
lv_obj_set_style_text_font(ui_Label7, &lv_font_montserrat_18, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_Screen3, ui_event_Screen3, LV_EVENT_ALL, NULL);

}