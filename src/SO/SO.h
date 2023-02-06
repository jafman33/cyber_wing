#ifndef __SO_H_INCLUDED__
#define __SO_H_INCLUDED__

#include "Arduino.h"
#include "SPI.h"
#include "AmberEncoder.h"

#include "../../SO_Config.h"
#include "../IMUs/IMUs.h"
#include "../BatteryMonitor/BatteryMonitor.h"
#include "../Steering/Steering.h"
#include "../Roboteq/Roboteq.h"
#include "../Elmo/Elmo.h"
#include "../SGFilter/SGFilter.h"
#include "../LowPassFilter/LowPassFilter.h"
#include "../CANFD/drv_canfdspi_api.h"
#include "../CANFD/drv_canfdspi_register.h"
#include "../CANFD/drv_spi.h"

namespace Cyberpod
{
	class SO
	{
		enum class STATUS : uint8_t
		{
			INIT     = 1,
			RUNNING  = 2,
			FAILURE  = 3
		};

		enum class CTRL_STATUS : uint8_t
		{
			RUNNING  = 1,
			TIMEDOUT = 2,
			FAILURE  = 3,
			CHKSUM_ERR = 4,
			BAD_IDX = 5
		};
	public:
		SO(void);
		void init(void);
		void update(void);

	protected:
		Elmo LEFT_ELMO_;
		Elmo RIGHT_ELMO_;
		YOST_TTS_LX imu_;
		STATUS status_;
		bool publishing_;
		BatteryMonitor batteryMonitor_;
		Steering steering_;
		Encoder encL_;
		Encoder encR_;
		LowPassFilter filterL_;
		LowPassFilter filterR_;
		Encoder_t encDataL_;
		Encoder_t encDataR_;
		uint32_t iterT_;
		uint32_t iterTnm1_;
		uint32_t encLTnm1_;
		uint32_t encRTnm1_;
		uint32_t iterCount_;
		uint32_t displayTnm1_;
		float state_[7];
		float input_[2];
		uint8_t errorMessage_;
		bool overrun_;
		uint32_t ctrlMsgErr_;
		CTRL_STATUS ctrlStatus_;
		bool idMotor_;

		float d_psi_filter = 0;
		float bias = 0;

		int32_t updateState(void);
		int32_t updateCtrl(void);
		void publishState(void);
		void forwardCtrl(void);

	};
}

#endif
