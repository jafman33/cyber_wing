#ifndef __SO_CONFIG_H_INCLUDED__
#define __SO_CONFIG_H_INCLUDED__

#include <vector>

union num32_t
{
	int32_t i;
	uint32_t ui;
	float f;
	uint8_t c[4];
};

union num16_t
{
	int16_t i;
	uint16_t ui;
	uint8_t c[2];
};

//////////////SO_OPTIONS/////////////////
#define SO_FAILURE_DT  100000
#define SO_DISPLAY_DT 1000000 //us
#define SO_CAN_PORT Can0
#define SO_CAN_BAUD 1000000
#define SO_CAN_DELAY 150 //us
#define SO_DT_TRAGET_LOW 500 //in microseconds
#define SO_DT_TRAGET_HIGH 1500 //in microseconds
#define SO_CTRL_TIMEOUT 2000 //in microseconds
#define SO_CTRL_MAX_CHKSUM_ERR 5
#define SO_UMAX 20.0F
/////////////////////////////////////////

//////////////////CANFD//////////////////
#define CANFD_PIN_SS 10
#define CANFD_PIN_INT 16
#define CANFD_PIN_INT0 17
#define CANFD_PIN_INT1 18
#define CANFD_UPDATE_TIMEOUT 500
/////////////////////////////////////////

////////////////FBL2360//////////////////
#define FBL_READ_TIMEOUT 5000 //us
#define FBL_RECEIVE_TIMEOUT 1500 //us
#define FBL_MODE Roboteq::MODE::CAN
/////////////////////////////////////////

/////////////////ELMO////////////////////
#define ELMO_MOTOR_LEFT_IDX 125 //us
#define ELMO_MOTOR_RIGHT_IDX 126 //us
#define ELMO_RECEIVE_TIMEOUT 5000 //us
#define ELMO_BAUD 1000000
/////////////////////////////////////////

//////////////YOST_TTS_LX////////////////
#define YOST_TTS_LX_PIN_SS 9
#define YOST_TTS_LX_PIN_ATT 19
#define YOST_TTS_LX_RECEIVE_TIMEOUT 100000000//10000
#define YOST_TTS_LX_RESET_SETTINGS true
#define YOST_TTS_LX_MODE IMUAbstract::MODE::FUSIONED // IMUAbstract::MODE::FUSIONED or IMUAbstract::MODE::RAW
/////////////////////////////////////////

/////////////BATTERY_SENSOR//////////////
#define BATTERY_CURRENT_PIN A9
#define BATTERY_CURRENT_TAU 1.0F //in seconds
#define BATTERY_VOLTAGE_PIN A0
#define BATTERY_VOLTAGE_TAU 1.0F //in seconds
/////////////////////////////////////////

/////////////STEERING_SENSOR/////////////
#define STEERING_PIN A1
#define STEERING_TAU 0.1F //in seconds
/////////////////////////////////////////

/////////////////ENCODERS////////////////
#define ENCODER_PIN_A_LEFT 25
#define ENCODER_PIN_B_LEFT 24
#define ENCODER_PIN_A_RIGHT 26
#define ENCODER_PIN_B_RIGHT 27
#define ENCODER_CPR_WHEEL 40000
#define ENCODER_TIMEOUT 100000
/////////////////////////////////////////

////////////////PARAMETERS///////////////
#define L 0.5F
#define Rw 0.195F
/////////////////////////////////////////

//////////////////FILTERS////////////////
#define FILTER_ORDER 2
#define FILTER_SIZE 10
/////////////////////////////////////////

//////////////////FILTERS////////////////
#define LPFILTER_TAU 0.005F
#define LPFILTER_TAU_DER 0.005F
/////////////////////////////////////////

#endif
