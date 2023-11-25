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
#define CONFIG_SLEEP_SPEED 10000
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

bool debugMode = true;
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
        if(debugMode){
            Serial.print("Putting Device To Sleep");
            //sleep_goto_dormant_until_edge_high(23);
            Serial.print(" at ");
            Serial.print(CONFIG_SLEEP_SPEED);
            Serial.print(" kHz while actual is ");
            Serial.println(rp2040.f_cpu()/1000);
            
        }
	}
    if(debugMode){
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
    //Common rounding technique
    uint8_t voltage_mv = uint8_t(state.battery.voltage + 0.5) * 1000;
    //4200 mv = 4.2V, max possible battery for lithium battery
    if(voltage_mv >= 4200){
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
							200 - tmp);

	// count += 60000;
	if (count > oneday) {
		count -= oneday;
	}
	if(!isDisplaySleep){
		drawClock(count);
	}
}
void wakeupcall(){
    // This helps obtain acceleration and other data for wake up
    // I used to use this to wake up the display
    // Untill I figured out how the interrupt works
    // This is now obsolete
	float accX = state.imu.acc.x;
	float accY = state.imu.acc.y;
	float accZ = state.imu.acc.z;
	
	float gX = state.imu.gyro.x;
	float gY = state.imu.gyro.y;
	float gZ = state.imu.gyro.z;
    
    delay(1);
	imuTick();

	float sqMagAcc = sq(accX - state.imu.acc.x) + sq(accY - state.imu.acc.y) + sq(accZ - state.imu.acc.z);
	float sqMagGyro = sq(gX - state.imu.gyro.x) + sq(gY - state.imu.gyro.y) + sq(gZ - state.imu.gyro.z);
	if(debugMode){
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
    //We set the frequency to a higher frequency
    set_sys_clock_khz(targetFrequency, false);
    //We log these and compare the actual possible frequency in debug mode
    if(debugMode){
        Serial.print("Setting Frequency to ");
        Serial.print(targetFrequency);
        Serial.print(" kHz while actual is ");
        Serial.println(rp2040.f_cpu()/1000);
    }
    //Reset sleep counter
    SAFE_STATE_UPDATE(&stateMtx, { state.counter = 0; });
    //Mark as awaken
    isDisplaySleep = false;
    //Delay a bit and tick, this make sure that the correct time 
    //will be displayed when the display is back on
    delay(50);
    displayTick();
    delay(50);
    //Wake up the display and set brightness
    display.wakeup();
    display.setBrightness(brightness);
}
void displaySetup(){
	// Calculate zoom factor to fit the display within the given width
    zoom = (float)(std::min(display.width(), display.height())) / width;

    // Set the pivot (rotation center) of the display to its center
    display.setPivot(display.width() >> 1, display.height() >> 1);

    // Set color depth for various components to 4-bit palette mode
    canvas.setColorDepth(lgfx::palette_4bit); 
    clockbase.setColorDepth(lgfx::palette_4bit);
    needle1.setColorDepth(lgfx::palette_4bit);
    shadow1.setColorDepth(lgfx::palette_4bit);
    needle2.setColorDepth(lgfx::palette_4bit);
    shadow2.setColorDepth(lgfx::palette_4bit);

    // Initialize the palette with grayscale colors (0: black, 15: white)
    // Colors 1-14 change gradually from black to white

    // Create sprites with specified width and height
    canvas.createSprite(width, width);  
    clockbase.createSprite(width, width);
    needle1.createSprite(9, 119);
    shadow1.createSprite(9, 119);
    needle2.createSprite(3, 119);
    shadow2.createSprite(3, 119);

    // Fill the sprites with a transparent color (initially filled with 0)
    canvas.fillScreen(transpalette);
    clockbase.fillScreen(transpalette);
    needle1.fillScreen(transpalette);
    shadow1.fillScreen(transpalette);

    // Set font and alignment for drawing numbers on the clock face
    clockbase.setTextFont(4); 
    clockbase.setTextDatum(lgfx::middle_center);

    // Draw the clock face background and outer circle
    clockbase.fillCircle(halfwidth, halfwidth, halfwidth, 1);
    clockbase.drawCircle(halfwidth, halfwidth, halfwidth - 1, 0);

    // Draw minute marks and numbers on the clock face
    for (int i = 1; i <= 60; ++i) {
        float rad = i * 6 * -0.0174532925; // Convert degrees to radians for positioning
        float cosy = -cos(rad) * (halfwidth * 10 / 11);
        float sinx = -sin(rad) * (halfwidth * 10 / 11);
        bool flg = 0 == (i % 5); // Flag for drawing hour marks

        // Draw the minute/hour marks
        clockbase.fillCircle(halfwidth + sinx + 1, halfwidth + cosy + 1, flg * 3 + 1, 4);
        clockbase.fillCircle(halfwidth + sinx, halfwidth + cosy, flg * 3 + 1, 12);

        // Draw numbers at hour marks
        if (flg) {
            cosy = -cos(rad) * (halfwidth * 10 / 13);
            sinx = -sin(rad) * (halfwidth * 10 / 13);
            clockbase.setTextColor(1);
            clockbase.drawNumber(i / 5, halfwidth + sinx + 1, halfwidth + cosy + 4);
            clockbase.setTextColor(15);
            clockbase.drawNumber(i / 5, halfwidth + sinx, halfwidth + cosy + 3);
        }
    }
    clockbase.setTextFont(7);

    // Set pivot points for the clock hands and their shadows
    needle1.setPivot(4, 100);
    shadow1.setPivot(4, 100);
    needle2.setPivot(1, 100);
    shadow2.setPivot(1, 100);

    // Draw the clock hands and their shadows
    for (int i = 6; i >= 0; --i) {
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

    // Set initial brightness and start writing to the display
    display.setBrightness(32);
    display.startWrite();
}
void SerialAdjustTime(){
    // Check if there is any data available to read from the Serial buffer
    if (!Serial.available()){
        return; // If no data is available, exit the function
    }
    // Read the input string from Serial until a newline character is encountered
    String input = Serial.readStringUntil('\n');
    // Find the position of the colon character in the input string
    int colonIndex = input.indexOf(':');
    // Check if the colon character was found
    if (colonIndex != -1) {
        // Extract the hours part from the input string
        String hoursStr = input.substring(0, colonIndex);
        // Extract the minutes part from the input string
        String minutesStr = input.substring(colonIndex + 1);
        // Trim whitespace from the extracted strings
        hoursStr.trim();
        minutesStr.trim();
        // Convert the hours and minutes to integers and calculate the total seconds
        int totalSeconds = (hoursStr.toInt() * 3600) + (minutesStr.toInt() * 60);
        // Convert total seconds to milliseconds and assign it to 'count'
        count = totalSeconds * 1000;
    } 
    else {
        // If no colon character was found, print an error message
        Serial.println("Invalid input format! Please use 'hh:mm' format.");
    }
}

void WOM_Interrupt(uint gpio, uint32_t events){
    // This function is an Interrupt Service Routine (ISR) for Wake on Motion (WOM).

    // Check if the correct interrupt event has occurred
    // Specifically, check if the interrupt is a rising edge event and it's from the expected GPIO pin
    if (!(events & GPIO_IRQ_EDGE_RISE && gpio == IMU_INTERRPUT_1)) {
        return; // If the event is not the one we're interested in, exit the function
    }
    // Check if the display is already awake
    if (!isDisplaySleep) {
        return; // If the display is not in sleep mode, no need to wake it up
    }
    // Brief delay to stabilize after the interrupt
    delay(10);
    // If the display is not a sleep possibly we should force it to sleep
    // display.sleep();
    // Additional delay to ensure the display is ready to be woken up
    // I suddenly don't see why we need this
    // sleep_ms(100);
    // Wake up the system with the specified configuration
    quickWakeup(CONFIG_SPEED, 32);
    // Mark the display as awake
    isDisplaySleep = false;
    // Log that a forced wakeup has occurred
    Serial.println("Forced wakeup from interrupt");
}


void setup() {
    // The serial communication setup
    Serial.begin(115200);
    //Uncomment this to makesure the device start working untill connected
    //Makes life easier
    //while(!Serial);
    Serial.println("Watchface");
    delay(100);
    //setup interrupt with the imu
    gpio_init(IMU_INTERRPUT_1);
    gpio_set_dir(IMU_INTERRPUT_1, GPIO_IN);
    gpio_pull_down(IMU_INTERRPUT_1);
    gpio_set_irq_enabled_with_callback(IMU_INTERRPUT_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &WOM_Interrupt);
	//pinMode(mainButton, OUTPUT);
	set_sys_clock_khz(CONFIG_SPEED, true);
  	// set up state access mutex
  	mutex_init(&stateMtx);
  	// initialize battery adc
  	battery.begin(PIN_BAT_ADC);

  	// initialize IMU
    // set up I2C1
    Wire1.setSDA(PIN_IMU_SDA);
    Wire1.setSCL(PIN_IMU_SCL);
    Wire1.setClock(400'000);
    Wire1.begin();

    // set up IMU driver
    state.imu.ready = imu.begin(&Wire1, QMI8658C_I2C_ADDRESS_PULLUP);
    imu.reset();
    sleep_ms(10);
    if (state.imu.ready) {
        imu.configureAcc(QMI8658C::AccScale::ACC_SCALE_4G, 
                        QMI8658C::AccODR::ACC_ODR_250HZ,
                        QMI8658C::AccLPF::ACC_LPF_5_32PCT);
        imu.configureGyro(QMI8658C::GyroScale::GYRO_SCALE_512DPS,
                            QMI8658C::GyroODR::GYRO_ODR_250HZ,
                            QMI8658C::GyroLPF::GYRO_LPF_5_32PCT);
        //attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT2), imuTick, PinStatus::RISING);
    }
    delay(1000);
	// initialize display
	display.begin(PIN_LCD_SCLK, PIN_LCD_MOSI, PIN_LCD_DC, PIN_LCD_CS, PIN_LCD_RST, PIN_LCD_BL);
    displaySetup();

	// attach timer callbacks for regular peripheral updates
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
    //This needs to be here for some reason, guess some time is required for this
    //to be properly setup.
    imu.QMI8658_enableWakeOnMotion();
}

void update7Seg(int32_t hour, int32_t min) {
    // Drawing the digital display part of the clock face

    // Set the font for the digital display
    clockbase.setTextFont(7);
    // Calculate the position for the digital display based on the pivot of the clockbase
    int x = clockbase.getPivotX() - 69; // Horizontal position
    int y = clockbase.getPivotY();      // Vertical position
    // Set the cursor to the calculated position
    clockbase.setCursor(x, y);
    // Set the color for erasing (overwriting) the previous display
    clockbase.setTextColor(5); 
    // Draw "88:88" to effectively clear the previous display
    clockbase.print("88:88");
    // Reset the cursor to the same position for the new display
    clockbase.setCursor(x, y);
    // Set the color for the actual display
    clockbase.setTextColor(12); 
    // Print the current hour and minute in "HH:MM" format
    clockbase.printf("%02d:%02d", hour, min);
}

void updateBattery() {
    // Set the dat of display to some small font, use 4 for larger font
	clockbase.setTextFont(3);
    // Obtain the battery percentage in percentage scale
	auto bat_pct = state.battery.percentage * 100;
    // Set color of the text
	clockbase.setTextColor(12);
    // Set piviot, where we will start working
	int x = clockbase.getPivotX() - 30;
	int y = clockbase.getPivotY() + 60;
	clockbase.setCursor(x, y);
    // Clears out an area for drawing text
	clockbase.fillRect(x, y - 15, 65, 25, 1);
	// Print both the voltage and the percentage
    // Comment one of these when using larger font
	clockbase.printf("%.1f V ", state.battery.voltage);
	clockbase.printf("%.0f%%", bat_pct);
}

void drawDot(int pos, int palette) {
    //Through calculation, draw the dots between the clock
	bool flg = 0 == (pos % 5);            
	float rad = pos * 6 * -0.0174532925;  
	float cosy = -cos(rad) * (halfwidth * 10 / 11);
	float sinx = -sin(rad) * (halfwidth * 10 / 11);
	canvas.fillCircle(halfwidth + sinx, halfwidth + cosy, flg * 3 + 1, palette);
}

void drawClock(uint64_t time) {
    // Drawing the clock

    // Previous minute value, used to check if the minute has changed
    static int32_t p_min = -1;

    // Calculate the current second and minute
    int32_t sec = time / 1000; // Convert milliseconds to seconds
    int32_t min = sec / 60;    // Convert seconds to minutes

    // If the minute has changed since the last update
    if (p_min != min) {
        p_min = min; // Update the previous minute tracker
        // Update the 7-segment display with the new hour and minute
        update7Seg(min / 60, min % 60);
    }

    // Update the battery status display
    updateBattery();

    // Draw the clock base sprite to the display buffer
    clockbase.pushSprite(0, 0);

    // Draw dots for seconds, minutes, and hours on the clock face
    drawDot(sec % 60, 14); // Draw the second dot
    drawDot(min % 60, 15); // Draw the minute dot
    drawDot(((min / 60) * 5) % 60, 15); // Draw the hour dot

    // Calculate angles for hour, minute, and second hands
    float fhour = (float)time / 120000;  // Hour hand angle
    float fmin = (float)time / 10000;    // Minute hand angle
    float fsec = (float)time * 6 / 1000; // Second hand angle

    // Get the pivot coordinates of the canvas
    int px = canvas.getPivotX();
    int py = canvas.getPivotY();

    // Draw shadows of the clock hands, offset slightly for a 3D effect
    shadow1.pushRotateZoom(px + 2, py + 2, fhour, 1.0, 0.7, transpalette); // Hour hand shadow
    shadow1.pushRotateZoom(px + 3, py + 3, fmin, 1.0, 1.0, transpalette);  // Minute hand shadow
    shadow2.pushRotateZoom(px + 4, py + 4, fsec, 1.0, 1.0, transpalette);  // Second hand shadow

    // Draw the actual clock hands
    needle1.pushRotateZoom(fhour, 1.0, 0.7, transpalette); // Hour hand
    needle1.pushRotateZoom(fmin, 1.0, 1.0, transpalette);  // Minute hand
    needle2.pushRotateZoom(fsec, 1.0, 1.0, transpalette);  // Second hand

    // Draw the entire canvas to the LCD, applying the zoom factor
    canvas.pushRotateZoom(0, zoom, zoom, transpalette);

    // Display the final image on the screen
    display.display();
}



bool highlow = true;
void loop() {
    Serial.print("Interrupt Read: ");
    Serial.println(digitalRead(IMU_INTERRPUT_1));
    batteryTick();
    //imuTick();
    //wakeupcall();
    //digitalRead(mainButton);
    highlow = !highlow;
    //digitalWrite(mainButton, highlow);
    if(isPowered){
        SerialAdjustTime();
    }
    else{
        Serial.println("Is no longer powered at above 4.1V");
    }
    
    if(!isPowered){
        if(counter % 8 == 0){
            counter = counter % 8;
            SleepCounter();
        }
        counter += 1;
    }
    delay(250);
}

