/*******************************************************************************
 * LVGL Watchface
 *
 * Anchor escapement video source:
 * https://www.youtube.com/watch?v=d_pV8TGKfMc
 *
 * Image by macrovector on Freepik:
 * https://www.freepik.com/free-vector/watch-set-expensive-classic-clock-with-leather-metal-straps-illustration_13031503.htm
 *
 * Dependent libraries:
 * LVGL: https://github.com/lvgl/lvgl.git
 *
 * LVGL Configuration file:
 * Copy your_arduino_path/libraries/lvgl/lv_conf_template.h
 * to your_arduino_path/libraries/lv_conf.h
 *
 * In lv_conf.h around line 15, enable config file:
 * #if 1 // Set it to "1" to enable content
 *
 * Then find and set:
 * #define LV_COLOR_DEPTH     16
 * #define LV_TICK_CUSTOM     1
 *
 * For SPI/parallel 8 display set color swap can be faster, parallel 16/RGB screen don't swap!
 * #define LV_COLOR_16_SWAP   1 // for SPI and parallel 8
 * #define LV_COLOR_16_SWAP   0 // for parallel 16 and RGB
 ******************************************************************************/
#define DEBUG false


#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include "ui/ui.h"
#include "pico/stdlib.h"
#include <pico/mutex.h>
#include <pico/stdlib.h>
// project-provided libs
#include <qmi8658c.hpp>
#include <cst816s.h>
// project-provided includes
#include "state.h"
// project source
#include "battery.hpp"
//Time Stamp MACRO
#include "timemacro.hpp"

/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/
#include <Arduino_GFX_Library.h>
#define GFX_BL PIN_LCD_BL
/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_RPiPicoSPI(PIN_LCD_DC /* DC */, PIN_LCD_CS /* CS */, PIN_LCD_SCLK /* SCK */, PIN_LCD_MOSI /* MOSI */, 13 /* RST */, spi1 /* spi */);
/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_GC9A01(bus, 13, 0 /* rotation */, true /* IPS */);
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/


// use compile time
#define CONFIG_DISPLAY_UPDATE_RATE_HZ (2u)
#define CONFIG_BATTERY_UPDATE_RATE_HZ (1u)
#define CONFIG_WAKE_SPEED 240000
#define CONFIG_SLEEP_SPEED 30000
#define IMU_INTERRPUT_1 23
#define IMU_INTERRPUT_2 24
#define ONE_MINUTE_MS (60 * 1000)
#define ONE_HOUR_MS (60 * 60 * 1000)
#define TWELVE_HOUR_MS (12 * 60 * 60 * 1000)
static uint8_t conv2d(const char *p)
{
  uint8_t v = 0;
  return (10 * (*p - '0')) + (*++p - '0');
}
static unsigned long ms_offset;

//Double buffering
#define LV_HOR_RES_MAX          240  // Horizontal resolution of your display
#define LV_VER_RES_MAX          240  // Vertical resolution of your display
#define LV_COLOR_DEPTH          16   // Color depth used
// Calculate the size required for one buffer
#define BUFFER_SIZE (LV_HOR_RES_MAX * LV_VER_RES_MAX)
// Allocate memory for the buffers
static lv_color_t buf_1[BUFFER_SIZE];  // First buffer
//static lv_color_t buf_2[BUFFER_SIZE];  // Second buffer

static uint32_t screenWidth = 240;
static uint32_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

static bool isScreenTouched = false;

static uint8_t curr_anchor_idx = 0;
static int16_t curr_anchor_angle = 0;
static lv_obj_t *anchors[8];
static unsigned long previous_loop_ms;
static unsigned long previous_clock_ms;
#define Screen_Refresh_Period (1000 / 20)
#define Clock_Frequency (1)
#define Clock_Period (1000 / Clock_Frequency)

static bool isDisplaySleep = false;

static Battery battery;
static QMI8658C imu;
static CST816S touch;

mutex stateMtx;
mutex_t lvgl_mutex;
volatile state_t state = {
    .counter = 0,
    .imu =
    {
        .ready = false,
        .temp = 0.0f,
        .acc = {.x = 0.0f, .y = 0.0f, .z = 0.0f},
        .gyro = {.x = 0.0f, .y = 0.0f, .z = 0.0f},
    },
    .battery =
    {
        .voltage = 0.0f,
        .percentage = 0.0f,
    },
};

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    #if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    #else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    #endif
    lv_disp_flush_ready( disp );
}
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX = 0, touchY = 0;

    //tft.getTouch( &touchX, &touchY, 600 );
    touch.CST816S_Get_Point();
    touchX = touch.x_point;
    touchY = touch.y_point;

    bool touched = isScreenTouched;

    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        Serial.print("Touch X: ");
        Serial.print(touchX);
        Serial.print(", Touch Y: ");
        Serial.println(touchY);
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
        /*
        Serial.print( "Data x " );
        Serial.println( touchX );

        Serial.print( "Data y " );
        Serial.println( touchY );
        */
        isScreenTouched = false;
    }
}
void my_log_cb(const char *buf)
{
  Serial.println(buf);
}

void batteryTick() {
	battery.update();
	SAFE_STATE_UPDATE(&stateMtx, {
		battery.voltage((float*)&state.battery.voltage);
		battery.percentage((float*)&state.battery.percentage);
	});
    
    Serial.print("Battery Voltage ");
    Serial.print(state.battery.voltage);
    Serial.print(" Percentage ");
    Serial.println(state.battery.percentage);
}
void setBrightness(float brightness){
    analogWrite(GFX_BL, brightness);
}
void quickWakeup(int targetFrequency, int brightness){
    set_sys_clock_khz(targetFrequency, false);
    Serial.print("Setting Frequency to ");
    Serial.print(targetFrequency);
    Serial.print(" kHz while actual is ");
    Serial.println(rp2040.f_cpu()/1000);
    SAFE_STATE_UPDATE(&stateMtx, { state.counter = 0; });
    delay(100);
    //displayTick();
    delay(50);
    gfx->displayOn();
    setBrightness(brightness);
}
void WakeUpCall(){
    if(!isDisplaySleep){
        return;
    }
    sleep_ms(10);
    gfx->displayOn();
    sleep_ms(10);
    quickWakeup(CONFIG_WAKE_SPEED, 32);
    Serial.println("Forced wakeup from interrupt");
}
void WOM_Interrput(uint gpio, uint32_t events){
    //Set display and clock back on
    Serial.println("Wom Interrupt triggered");
    if(!(events&GPIO_IRQ_EDGE_RISE && gpio == IMU_INTERRPUT_1)){
        return;
    }
    WakeUpCall();
}
void Touch_Interrput(uint gpio, uint32_t events){
    isScreenTouched = true;
    //Set display and clock back on
    if(!(events&GPIO_IRQ_EDGE_RISE && gpio == Touch_INT_PIN)){
        return;
    }
    WakeUpCall();
}
void SleepCounter(){
	int sleepTime = 10;

	if(state.counter >= sleepTime){
		SAFE_STATE_UPDATE(&stateMtx, { state.counter = sleepTime; });
		gfx->displayOff();
		isDisplaySleep = true;
		set_sys_clock_khz(CONFIG_SLEEP_SPEED, false);
        Serial.print("Putting Device To Sleep");
        //sleep_goto_dormant_until_edge_high(23);
        Serial.print(" at ");
        Serial.print(CONFIG_SLEEP_SPEED);
        Serial.print(" kHz while actual is ");
        Serial.println(rp2040.f_cpu()/1000);
	}
    Serial.print("State Counter");
    Serial.println(state.counter);
	SAFE_STATE_UPDATE(&stateMtx, { state.counter += 1; });
}
void setup()
{

    Serial.begin(115200);
    //Serial.setDebugOutput(true);

    if(DEBUG){
        while(!Serial);
        delay(100);
        Serial.println("LVGL Watchface");
        delay(100);
    }
    
    


    //pinMode(mainButton, OUTPUT);
	set_sys_clock_khz(CONFIG_WAKE_SPEED, true);
  	// set up state access mutex
  	mutex_init(&stateMtx);
    mutex_init(&lvgl_mutex);
    // Init Battery Reading
    battery.begin(PIN_BAT_ADC);
    // Init Display
    gfx->begin();
    gfx->fillScreen(BLACK);
    Serial.println("Display setup and filled with black");
    gfx->fillScreen(WHITE);
    // set up I2C1
    Wire1.setSDA(PIN_IMU_SDA);
    Wire1.setSCL(PIN_IMU_SCL);
    Wire1.setClock(400'000);
    Wire1.begin();
    // Brightness Setup
    gpio_init(GFX_BL);
    // Touch Setup
    touch.CST816S_init(CST816S_Point_Mode, &Wire1);
    gpio_init(Touch_INT_PIN);
    gpio_set_dir(Touch_INT_PIN, GPIO_IN);
    gpio_pull_up(Touch_INT_PIN);
    gpio_set_irq_enabled_with_callback(Touch_INT_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &Touch_Interrput);
    Serial.println("Touch Setup Complete");
    // IMU setup
    state.imu.ready = imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
    //gpio_init(IMU_INTERRPUT_1);
    //gpio_set_dir(IMU_INTERRPUT_1, GPIO_IN);
    //gpio_pull_down(IMU_INTERRPUT_1);
    //gpio_set_irq_enabled_with_callback(IMU_INTERRPUT_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &WOM_Interrput);
    //imu.QMI8658_enableWakeOnMotion();
    Serial.println("IMU Setup Complete");
    // use compile time for demo only
    uint8_t hh = conv2d(__TIME__);
    uint8_t mm = conv2d(__TIME__ + 3);
    uint8_t ss = conv2d(__TIME__ + 6);
    ms_offset = ((60 * 60 * hh) + (60 * mm) + ss) * 1000;

    #ifdef GFX_EXTRA_PRE_INIT
        GFX_EXTRA_PRE_INIT();
    #endif


    #ifdef GFX_BL
        pinMode(GFX_BL, OUTPUT);
        // digitalWrite(GFX_BL, HIGH);
        analogWrite(GFX_BL, 127);
    #endif

    #if LV_USE_LOG
        lv_log_register_print_cb(my_log_cb);
    #endif

    lv_init();
    Serial.println("UI Initialization");

    screenWidth = gfx->width();
    screenHeight = gfx->height();
    //#ifdef ESP32
        //disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 32, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    //#else
        //disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * 32);
    //#endif

    //lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * 32);
    //Do double buffering instead
    lv_disp_draw_buf_init(&draw_buf, buf_1, NULL, screenWidth * screenHeight);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    /* Init SquareLine prepared UI */
    ui_init();

    Serial.println("Setup done");
}
void loop()
{
    uint32_t status;
    uint32_t time_till_next = lv_timer_handler(); /* let the GUI do its work */
    //Serial.println(time_till_next);

    unsigned long ms = millis();

    unsigned long elapsed = ms - previous_loop_ms;
    if(elapsed >= 1){
        previous_loop_ms = ms;
        lv_tick_inc(elapsed);
    }

    // handle anchor escapement movement
    if (ms - previous_clock_ms >= Clock_Period)
    {
        batteryTick();
        previous_clock_ms = ms;
        // set watch arms' angle
        unsigned long clock_ms = (ms_offset + ms) % TWELVE_HOUR_MS;
        uint8_t hour = clock_ms / ONE_HOUR_MS;
        uint8_t minute = (clock_ms % ONE_HOUR_MS) / ONE_MINUTE_MS;
        int16_t angle = (clock_ms % ONE_MINUTE_MS) * 3600 / ONE_MINUTE_MS;
        lv_img_set_angle(ui_ImageArmSecond, angle);
        angle = (angle + (minute * 3600)) / 60;
        lv_img_set_angle(ui_ImageArmMinute, angle);
        angle = (angle + (hour * 3600)) / 12;
        lv_img_set_angle(ui_ImageArmHour, angle);
        char timeStr[7], voltStr[5];
        sprintf(timeStr, "%02d  %02d", hour, minute);
        sprintf(voltStr, "%.1fV", state.battery.voltage);
        lv_label_set_text(ui_DigitalTimeLabel, timeStr);
        lv_label_set_text(ui_BatteryVoltageLabel, voltStr);
        lv_arc_set_value(ui_BatteryPercentageArc, state.battery.percentage * 100);
    }
    setBrightness(lv_arc_get_value(ui_BrightnessArc));
    //Serial.print(lv_arc_get_value(ui_BrightnessArc));
    sleep_ms(time_till_next);
    
}