/*****************************************************************************
 * | File      	:   CST816S.h
 * | Author      :   Waveshare team
 * | Function    :   Hardware underlying interface
 * | Info        :
 *                Used to shield the underlying layers of each master
 *                and enhance portability
 *----------------
 * |	This version:   V1.0
 * | Date        :   2022-12-02
 * | Info        :   Basic version
 *
 ******************************************************************************/
#ifndef __CST816S_H
#define __CST816S_H

#include <Arduino.h>
#include <Wire.h>
#define CST816_ADDR (0x15)

typedef enum
{

	CST816_GestureID = 0x01,
	CST816_FingerNum,
	CST816_XposH,
	CST816_XposL,
	CST816_YposH,
	CST816_YposL,

	CST816_ChipID = 0xA7,
	CST816_ProjID,
	CST816_FwVersion,
	CST816_MotionMask,

	CST816_BPC0H = 0xB0,
	CST816_BPC0L,
	CST816_BPC1H,
	CST816_BPC1L,

	CST816_IrqPluseWidth = 0xED,
	CST816_NorScanPer,
	CST816_MotionSlAngle,
	CST816_LpScanRaw1H =0XF0,
	CST816_LpScanRaw1L,
	CST816_LpScanRaw2H,
	CST816_LpScanRaw2L,
	CST816_LpAutoWakeTime,
	CST816_LpScanTH,
	CST816_LpScanWin,
	CST816_LpScanFreq,
	CST816_LpScanIdac,
	CST816_AutoSleepTime,
	CST816_IrqCtl,
	CST816_AutoReset,
	CST816_LongPressTime,
	CST816_IOCtl,
	CST816_DisAutoSleep
} CST816S_Register;

/**
 * Whether the graphic is filled
 **/
typedef enum
{
	CST816S_Point_Mode = 1,
	CST816S_Gesture_Mode,
	CST816S_ALL_Mode,
} CST816S_Mode;

typedef enum
{
	CST816S_Gesture_None = 0,
	CST816S_Gesture_Up,
	CST816S_Gesture_Down,
	CST816S_Gesture_Left,
	CST816S_Gesture_Right,
	CST816S_Gesture_Click,
	CST816S_Gesture_Double_Click = 0x0b,
	CST816S_Gesture_Long_Press=0x0c,
} CST816S_Gesture;


class CST816S
{
    TwoWire* m_wire;
    public:
	uint16_t x_point;
	uint16_t y_point;
	uint8_t Gesture;
	uint8_t mode;

    uint8_t CST816S_init(uint8_t mode, TwoWire* wire);
    CST816S CST816S_Get_Point();
    uint8_t CST816S_Get_Gesture(void);
    void CST816S_I2C_Write(uint8_t reg, uint8_t value);
    uint8_t CST816S_I2C_Read(uint8_t reg);
    uint8_t CST816S_WhoAmI();
    void CST816S_Reset();
    uint8_t CST816S_Read_Revision();
    void CST816S_Wake_up();
    void CST816S_Stop_Sleep();
    void CST816S_Set_Mode(uint8_t mode);

};

extern CST816S Touch_CTS816;

#endif