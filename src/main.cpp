#include "pico/stdlib.h"

#include <Arduino.h>
#include <Wire.h>
#include <pico/mutex.h>
#include <pico/stdlib.h>

// project-provided libs
#include <qmi8658c.hpp>

// project-provided includes
#include "state.h"

// project source
#include "battery.hpp"
#include "lgfx_gc9a01.hpp"

//Time Stamp MACRO
#include "timemacro.hpp"


//
// application config
//
#define CONFIG_DISPLAY_UPDATE_RATE_HZ (2u)
#define CONFIG_BATTERY_UPDATE_RATE_HZ (2u)
#define CONFIG_WAKEUP_UPDATE_RATE_HZ (4u)
#define CONFIG_SPEED 30000
#define CONFIG_SLEEP_SPEED 1000
#define IMU_INTERRPUT_1 23
#define IMU_INTERRPUT_2 24


bool isDisplaySleep = false;
int mainButton = 26;
unsigned long lastCheckTime = 0;

//
// application state
//
mutex stateMtx;

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

static Battery battery;
static QMI8658C imu;

static LGFX_GC9A01 display;
static LGFX_Sprite canvas(&display);
static LGFX_Sprite clockbase(&canvas);
static LGFX_Sprite needle1(&canvas);
static LGFX_Sprite shadow1(&canvas);
static LGFX_Sprite needle2(&canvas);
static LGFX_Sprite shadow2(&canvas);

static constexpr uint64_t oneday = 86400000;
static uint64_t count = (__TIME_HOURS__ * SEC_PER_HOUR + __TIME_MINUTES__ * SEC_PER_MIN + __TIME_SECONDS__ + 15) * 1000;
static int32_t width = 239;
static int32_t halfwidth = width >> 1;
static auto transpalette = 0;
static float zoom;


int counter = 0;

repeating_timer timerDisplay;
repeating_timer timerBattery;

bool SerialMode = false;
bool isPowered = false;


void drawDot(int pos, int palette);
void drawClock(uint64_t time);
void batteryTick();
void imuTick();
void displayTick();
void wakeupcall();
void quickWakeup(int targetFrequency, int brightness);
void WOM_Interrput(uint gpio, uint32_t events);

void SleepCounter(){
	int sleepTime = 10;

    if(isPowered){
        return; //No not sleep when high input
    }

	if(state.counter >= sleepTime){
		SAFE_STATE_UPDATE(&stateMtx, { state.counter = sleepTime; });
		display.sleep();
		isDisplaySleep = true;
		set_sys_clock_khz(CONFIG_SLEEP_SPEED, false);
        if(SerialMode){
            Serial.print("Putting Device To Sleep");
            //sleep_goto_dormant_until_edge_high(23);
            Serial.print(" at ");
            Serial.print(CONFIG_SLEEP_SPEED);
            Serial.print(" kHz while actual is ");
            Serial.println(rp2040.f_cpu()/1000);
            
        }
	}
    if(SerialMode){
        Serial.print("State Counter");
        Serial.println(state.counter);
    }
	SAFE_STATE_UPDATE(&stateMtx, { state.counter += 1; });
}
void batteryTick() {
	battery.update();
	SAFE_STATE_UPDATE(&stateMtx, {
		battery.voltage((float*)&state.battery.voltage);
		battery.percentage((float*)&state.battery.percentage);
	});
    
    if(state.battery.voltage >= 4.15){
        if(!isPowered){
            quickWakeup(100000, 128);
        }
        isPowered = true;
    }else{
        if(isPowered){
            quickWakeup(CONFIG_SPEED, 32);
        }
        isPowered = false;
    }
}
void imuTick() {
	SAFE_STATE_UPDATE(&stateMtx, {
		if (state.imu.ready) {
		imu.readTemperature((float*)&state.imu.temp);
		imu.readAccelerometer((float*)&state.imu.acc.x, (float*)&state.imu.acc.y,
								(float*)&state.imu.acc.z);
		imu.readGyroscope((float*)&state.imu.gyro.x, (float*)&state.imu.gyro.y,
							(float*)&state.imu.gyro.z);
		}
	});
}
void displayTick() {

	static uint32_t p_milli = 0;
	uint32_t milli = lgfx::millis() % 1000;
	if (p_milli < milli)
		count += (milli - p_milli);
	else
		count += 1000 + (milli - p_milli);
	p_milli = milli;

	int32_t tmp = (count % 1000) >> 3;
	canvas.setPaletteColor(8, 255 - (tmp >> 1), 255 - (tmp >> 1),
							200 - tmp);  // 秒針の描画色を変化させる
										// count += 60000;
	if (count > oneday) {
		count -= oneday;
	}
	
	if(!isDisplaySleep){
		drawClock(count);
	}
}
void wakeupcall(){
	float accX = state.imu.acc.x;
	float accY = state.imu.acc.y;
	float accZ = state.imu.acc.z;
	
	float gX = state.imu.gyro.x;
	float gY = state.imu.gyro.y;
	float gZ = state.imu.gyro.z;
    
	imuTick();

	float sqMagAcc = sq(accX - state.imu.acc.x) + sq(accY - state.imu.acc.y) + sq(accZ - state.imu.acc.z);
	float sqMagGyro = sq(gX - state.imu.gyro.x) + sq(gY - state.imu.gyro.y) + sq(gZ - state.imu.gyro.z);
	if(SerialMode){
		Serial.print("Acceleration, ");
		Serial.print(sqMagAcc);
		Serial.print(" Gyro, ");
		Serial.println(sqMagGyro);
	}

    if(isPowered){
        return;
    }

	if(sqMagAcc > 4 && sqMagGyro > 200000){
        quickWakeup(CONFIG_SPEED, 32);
	}
}
void quickWakeup(int targetFrequency, int brightness){
    set_sys_clock_khz(targetFrequency, false);
    if(SerialMode){
        Serial.print("Setting Frequency to ");
        Serial.print(targetFrequency);
        Serial.print(" kHz while actual is ");
        Serial.println(rp2040.f_cpu()/1000);
    }
    SAFE_STATE_UPDATE(&stateMtx, { state.counter = 0; });
    isDisplaySleep = false;
    delay(200);
    displayTick();
    delay(50);
    display.wakeup();
    display.setBrightness(brightness);
}
void displaySetup(){
	zoom = (float)(std::min(display.width(), display.height())) /
           width;  // 表示が画面にフィットするよう倍率を調整

    display.setPivot(
        display.width() >> 1,
        display.height() >> 1);  // 時計描画時の中心を画面中心に合わせる

    canvas.setColorDepth(
        lgfx::palette_4bit);  // 各部品を４ビットパレットモードで準備する
    clockbase.setColorDepth(lgfx::palette_4bit);
    needle1.setColorDepth(lgfx::palette_4bit);
    shadow1.setColorDepth(lgfx::palette_4bit);
    needle2.setColorDepth(lgfx::palette_4bit);
    shadow2.setColorDepth(lgfx::palette_4bit);
    // パレットの初期色はグレースケールのグラデーションとなっており、
    // 0番が黒(0,0,0)、15番が白(255,255,255)
    // 1番～14番は黒から白へ段階的に明るさが変化している
    //
    // パレットを使う場合、描画関数は色の代わりに0～15のパレット番号を指定する

    canvas.createSprite(width, width);  // メモリ確保
    clockbase.createSprite(width, width);
    needle1.createSprite(9, 119);
    shadow1.createSprite(9, 119);
    needle2.createSprite(3, 119);
    shadow2.createSprite(3, 119);

    canvas.fillScreen(
        transpalette);  // 透過色で背景を塗り潰す
                        // (create直後は0埋めされているので省略可能)
    clockbase.fillScreen(transpalette);
    needle1.fillScreen(transpalette);
    shadow1.fillScreen(transpalette);

    clockbase.setTextFont(4);  // フォント種類を変更(時計盤の文字用)
    clockbase.setTextDatum(lgfx::middle_center);
    clockbase.fillCircle(halfwidth, halfwidth, halfwidth,
                         1);  // 時計盤の背景の円を塗る
    clockbase.drawCircle(halfwidth, halfwidth, halfwidth - 1, 0);
    for (int i = 1; i <= 60; ++i) {
		float rad = i * 6 * -0.0174532925;  // 時計盤外周の目盛り座標を求める
		float cosy = -cos(rad) * (halfwidth * 10 / 11);
		float sinx = -sin(rad) * (halfwidth * 10 / 11);
		bool flg = 0 == (i % 5);  // ５目盛り毎フラグ
		clockbase.fillCircle(halfwidth + sinx + 1, halfwidth + cosy + 1,
							flg * 3 + 1, 4);  // 目盛りを描画
		clockbase.fillCircle(halfwidth + sinx, halfwidth + cosy, flg * 3 + 1, 12);
		if (flg) {  // 文字描画
			cosy = -cos(rad) * (halfwidth * 10 / 13);
			sinx = -sin(rad) * (halfwidth * 10 / 13);
			clockbase.setTextColor(1);
			clockbase.drawNumber(i / 5, halfwidth + sinx + 1, halfwidth + cosy + 4);
			clockbase.setTextColor(15);
			clockbase.drawNumber(i / 5, halfwidth + sinx, halfwidth + cosy + 3);
		}
    }
    clockbase.setTextFont(7);

    needle1.setPivot(4, 100);  // 針パーツの回転中心座標を設定する
    shadow1.setPivot(4, 100);
    needle2.setPivot(1, 100);
    shadow2.setPivot(1, 100);

    for (int i = 6; i >= 0; --i) {  // 針パーツの画像を作成する
      	needle1.fillTriangle(4, -16 - (i << 1), 8, needle1.height() - (i << 1), 0,
                           	needle1.height() - (i << 1), 15 - i);
      	shadow1.fillTriangle(4, -16 - (i << 1), 8, shadow1.height() - (i << 1), 0,
                           	shadow1.height() - (i << 1), 1 + i);
    }
    for (int i = 0; i < 7; ++i) {
      	needle1.fillTriangle(4, 16 + (i << 1), 8,
                           	needle1.height() + 32 + (i << 1), 0,
                           	needle1.height() + 32 + (i << 1), 15 - i);
      	shadow1.fillTriangle(4, 16 + (i << 1), 8,
                           	shadow1.height() + 32 + (i << 1), 0,
                           	shadow1.height() + 32 + (i << 1), 1 + i);
    }
    needle1.fillTriangle(4, 32, 8, needle1.height() + 64, 0,
                         	needle1.height() + 64, 0);
    shadow1.fillTriangle(4, 32, 8, shadow1.height() + 64, 0,
                         	shadow1.height() + 64, 0);
    needle1.fillRect(0, 117, 9, 2, 15);
    shadow1.fillRect(0, 117, 9, 2, 1);
	
    needle1.drawFastHLine(1, 117, 7, 12);
    shadow1.drawFastHLine(1, 117, 7, 4);

    needle1.fillCircle(4, 100, 4, 15);
    shadow1.fillCircle(4, 100, 4, 1);
    needle1.drawCircle(4, 100, 4, 14);

    needle2.fillScreen(9);
    shadow2.fillScreen(3);
    needle2.drawFastVLine(1, 0, 119, 8);
    shadow2.drawFastVLine(1, 0, 119, 1);
    needle2.fillRect(0, 99, 3, 3, 8);

    display.setBrightness(32);
    display.startWrite();
}
void SerialAdjustTime(){
    if (!Serial.available()){
        return;
    }
    String input = Serial.readStringUntil('\n');
    int colonIndex = input.indexOf(':');
    if (colonIndex != -1) {
        String hoursStr = input.substring(0, colonIndex);
        String minutesStr = input.substring(colonIndex + 1);
        hoursStr.trim();
        minutesStr.trim();
        int totalSeconds = (hoursStr.toInt() * 3600) + (minutesStr.toInt() * 60);
        count = totalSeconds * 1000;
    } 
    else {
        Serial.println("Invalid input format! Please use 'hh:mm' format.");
    }
}
void WOM_Interrput(uint gpio, uint32_t events){
    //Set display and clock back on
    if(!(events&GPIO_IRQ_EDGE_RISE && gpio == IMU_INTERRPUT_1)){
        return;
    }
    if(!isDisplaySleep){
        return;
    }
    sleep_ms(10);
    display.sleep();
    sleep_ms(200);
    quickWakeup(CONFIG_SPEED, 32);
    isDisplaySleep = false;
    Serial.println("Forced wakeup from interrupt");
}

void setup() {
    Serial.begin(9600);
    //setup interrupt
    gpio_init(IMU_INTERRPUT_1);
    gpio_set_dir(IMU_INTERRPUT_1, GPIO_IN);
    gpio_pull_down(IMU_INTERRPUT_1);
    gpio_set_irq_enabled_with_callback(IMU_INTERRPUT_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &WOM_Interrput);
	
	//pinMode(mainButton, OUTPUT);
	set_sys_clock_khz(CONFIG_SPEED, true);
  	// set up state access mutex
  	mutex_init(&stateMtx);
  	// initialize battery adc
  	{ battery.begin(PIN_BAT_ADC); }

  	// initialize IMU
  	{
		// set up I2C1
		Wire1.setSDA(PIN_IMU_SDA);
		Wire1.setSCL(PIN_IMU_SCL);
		Wire1.setClock(400'000);
		Wire1.begin();

		// set up IMU driver
		state.imu.ready = imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
		if (state.imu.ready) {
			imu.configureAcc(QMI8658C::AccScale::ACC_SCALE_4G,
							QMI8658C::AccODR::ACC_ODR_250HZ,
							QMI8658C::AccLPF::ACC_LPF_5_32PCT);
			imu.configureGyro(QMI8658C::GyroScale::GYRO_SCALE_512DPS,
								QMI8658C::GyroODR::GYRO_ODR_250HZ,
								QMI8658C::GyroLPF::GYRO_LPF_5_32PCT);
			//attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT2), imuTick, PinStatus::RISING);
            imu.QMI8658_enableWakeOnMotion();
		}
  	}

	// initialize display
	{
		display.begin(PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS,
					PIN_LCD_RST, PIN_LCD_BL);
		displaySetup();
	}

	// attach timer callbacks for regular peripheral updates
	{
		// display
		add_repeating_timer_ms(
			-(1000 / CONFIG_DISPLAY_UPDATE_RATE_HZ),
			[](struct repeating_timer* t) {
			displayTick();
			return true;
			},
			NULL, &timerDisplay);
		
		/*
		// battery monitor
		add_repeating_timer_ms(
			-(1000 / CONFIG_BATTERY_UPDATE_RATE_HZ),
			[](struct repeating_timer* t) {
			batteryTick();
			return true;
			},
			NULL, &timerBattery);
		*/
	}
}

void update7Seg(int32_t hour, int32_t min) {  // 時計盤のデジタル表示部の描画
	clockbase.setTextFont(7);
	int x = clockbase.getPivotX() - 69;
	int y = clockbase.getPivotY();
	clockbase.setCursor(x, y);
	clockbase.setTextColor(5);  // 消去色で 88:88 を描画する
	clockbase.print("88:88");
	clockbase.setCursor(x, y);
	clockbase.setTextColor(12);  // 表示色で時:分を描画する
	clockbase.printf("%02d:%02d", hour, min);
}

void updateBattery() {
	clockbase.setTextFont(4);
	auto bat_pct = state.battery.percentage * 100;
	int x = clockbase.getPivotX() - 30;
	int y = clockbase.getPivotY() + 60;
	clockbase.setTextColor(12);
	clockbase.setCursor(x, y);
	clockbase.fillRect(x, y - 15, 65, 25, 2);
	// clockbase.printf("%.0f%%", bat_pct);
	clockbase.printf("%.1f V", state.battery.voltage);
}

void drawDot(int pos, int palette) {
	bool flg = 0 == (pos % 5);            // ５目盛り毎フラグ
	float rad = pos * 6 * -0.0174532925;  // 時計盤外周の目盛り座標を求める
	float cosy = -cos(rad) * (halfwidth * 10 / 11);
	float sinx = -sin(rad) * (halfwidth * 10 / 11);
	canvas.fillCircle(halfwidth + sinx, halfwidth + cosy, flg * 3 + 1, palette);
}

void drawClock(uint64_t time) {  // 時計の描画
	static int32_t p_min = -1;
	int32_t sec = time / 1000;
	int32_t min = sec / 60;
	if (p_min != min) {  // 分の値が変化していれば時計盤のデジタル表示部分を更新
		p_min = min;
		update7Seg(min / 60, min % 60);
	}
	updateBattery();
	clockbase.pushSprite(0, 0);  // 描画用バッファに時計盤の画像を上書き

	drawDot(sec % 60, 14);
	drawDot(min % 60, 15);
	drawDot(((min / 60) * 5) % 60, 15);

	float fhour = (float)time / 120000;   // 短針の角度
	float fmin = (float)time / 10000;     // 長針の角度
	float fsec = (float)time * 6 / 1000;  // 秒針の角度
	int px = canvas.getPivotX();
	int py = canvas.getPivotY();
	shadow1.pushRotateZoom(px + 2, py + 2, fhour, 1.0, 0.7,
							transpalette);  // 針の影を右下方向にずらして描画する
	shadow1.pushRotateZoom(px + 3, py + 3, fmin, 1.0, 1.0, transpalette);
	shadow2.pushRotateZoom(px + 4, py + 4, fsec, 1.0, 1.0, transpalette);
	needle1.pushRotateZoom(fhour, 1.0, 0.7, transpalette);  // 針を描画する
	needle1.pushRotateZoom(fmin, 1.0, 1.0, transpalette);
	needle2.pushRotateZoom(fsec, 1.0, 1.0, transpalette);

	canvas.pushRotateZoom(0, zoom, zoom,
							transpalette);  // 完了した時計盤をLCDに描画する
	display.display();
}


bool highlow = true;
void loop() {
    Serial.println(digitalRead(IMU_INTERRPUT_1));
    batteryTick();
    wakeupcall();
    //digitalRead(mainButton);
    highlow = !highlow;
    //digitalWrite(mainButton, highlow);
    if(isPowered){
        if(!SerialMode){
            SerialMode = true;
            Serial.begin(9600);
        }
        SerialAdjustTime();
    }
    else if(SerialMode){
        SerialMode = false;
        Serial.println("Is no longer powered at above 4.2V");
        //Serial.end();
    }
    
    if(!isPowered){
        if(counter % 4 == 0){
            counter = counter % 4;
            SleepCounter();
        }
        counter += 1;
    }
    delay(250);
}

