SET(SOURCES screens/ui_Screen1.c
    screens/ui_Screen2.c
    screens/ui_Screen3.c
    screens/ui_Screen4.c
    screens/ui_Screen5.c
    screens/ui_Screen6.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_watchface240_png.c
    images/ui_img_armhour_png.c
    images/ui_img_armminute_png.c
    images/ui_img_armsecond_png.c
    images/ui_img_out1_png.c
    images/ui_img_out2_png.c
    images/ui_img_out3_png.c
    images/ui_img_out4_png.c
    images/ui_img_out5_png.c
    images/ui_img_out6_png.c
    images/ui_img_out7_png.c
    images/ui_img_out8_png.c)

add_library(ui ${SOURCES})
