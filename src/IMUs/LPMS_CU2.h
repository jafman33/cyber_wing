#ifndef __LPMS_CU2_H_INCLUDED__
#define __LPMS_CU2_H_INCLUDED__

#include "Arduino.h"
#include "IMUAbstract.h"
#include "FlexCAN.h"

namespace Cyberpod
{

	class LPMS_CU2  : public IMUAbstract
	{
		enum class COMMAND : uint16_t
		{
			REPLY_ACK                = 0,
			REPLY_NACK               = 1,
			UPDATE_FIRMWARE          = 2,
			UPDATE_IAP               = 3,
			GET_CONFIG               = 4,
			GET_STATUS               = 5,
			GOTO_COMMAND_MODE        = 6,
			GOTO_STREAM_MODE         = 7,
			GET_SENSOR_DATA          = 9,
			SET_TRANSMIT_DATA        = 10,
			SET_STREAM_FREQ          = 11,
			SET_IMU_ID               = 20,
			GET_IMU_ID               = 21,
			SET_GYR_RANGE            = 25,
			GET_GYR_RANGE            = 26,
			SET_ACC_RANGE            = 31,
			GET_ACC_RANGE            = 32,
			SET_MAG_RANGE            = 33,
			GET_MAG_RANGE            = 34,
			SET_FILTER_MODE          = 41,
			GET_FILTER_MODE          = 42,
			SET_FILTER_PRESET        = 43,
			GET_FILTER_PRESET        = 44,
			SET_RAW_DATA_LP          = 60,
			GET_RAW_DATA_LP          = 61,
			RESET_TIMESTAMP          = 66,
			SET_LIN_ACC_COMP_MODE    = 67,
			GET_LIN_ACC_COMP_MODE    = 68,
			SET_CAN_CHANNEL_MODE     = 72,
			SET_CAN_POINT_MODE       = 73,
			SET_LPBUS_DATA_MODE      = 75,
			RESET_ORIENTATION_OFFSET = 82,
			SET_UART_BAUDRATE        = 84,
			GET_UART_BAUDRATE        = 85,
			SET_UART_FORMAT          = 86,			
		};

		struct CONFIG
		{
			uint16_t openMATID             = 0x01;
			int32_t streamFreq             = 400;
			int32_t rawStreamData          = 7168;
			int32_t fusionedStreamData     = 327680;
			int32_t uartFormat             = 0;
			int32_t busDataMode            = 0;
			int32_t canPointMode           = 1;
			int32_t canChannelMode         = 1;
			uint32_t rawMessageLength      = 7;
			uint32_t fusionedMessageLength = 6;
			float quatOffset0[4]           = {1.0F, 0.0F, 0.0F, 0.0F};
		};

	public:
		LPMS_CU2(const IMUAbstract::MODE &mode,
						 FlexCAN &port,
						 const uint32_t &baud,
		         const uint32_t &readTimeout,
		         const uint32_t &receiveTimeout,
		         const uint32_t &ackTimeout,
		         const uint32_t &resyncTimeout);
		virtual ~LPMS_CU2();

		virtual int32_t init(bool resetSettings = false);
		virtual int32_t update(void);
		int32_t startStream(void);
		int32_t stopStream(void);
		
	protected:
		FlexCAN &port_;
		const uint32_t baud_;
		const uint32_t readTimeout_;
		const uint32_t ackTimeout_;
		const uint32_t resyncTimeout_;
		const CONFIG config_;
		const uint32_t messageLength_;
		CAN_message_t msgOut_;
		CAN_message_t msgIn_;

		bool newMsgCheck(void);
		bool newStreamCheck(void);
		int32_t readMessage(COMMAND &cmd,
		                    std::vector<uint8_t> &data);
		int32_t sendMessage(const COMMAND &cmd,
						            const uint16_t &dataLen,
												const uint8_t data[]);
		int32_t gotoCommandMode(void);
		int32_t gotoStreamMode(void);
		int32_t setStreamFreq(void);
		int32_t setTransmitData(void);
		int32_t setCanChannelMode(void);
		int32_t setCanPointMode(void);	
		int32_t setUartFormat(void);
		int32_t setLpbusDataMode(void);
		int32_t tareReset(void);
		int32_t reSync(void);
		int32_t checkACK(void);
		int32_t parseStream(const std::vector<uint8_t> &data);

		bool isValidCommand(const uint16_t &cmd);

	};

}
#endif