#ifndef __BATTERY_H_INCLUDED__
#define __BATTERY_H_INCLUDED__

#include "Arduino.h"

namespace Cyberpod
{

	class BatteryMonitor
	{
	public:
		struct CONFIG
		{
			float mV_A = 264.0F;
			float mV_V = 54.828F;
			float Vref = 3300.0F; //in mV
		};

	public:
		BatteryMonitor(const int32_t &pinCurrent,
			             const int32_t &pinVoltage,
			             const float &currentFilterTau,
			             const float &voltageFilterTau);
		
		BatteryMonitor(const int32_t &pinCurrent,
			             const int32_t &pinVoltage,
			             const float &currentFilterTau,
			             const float &voltageFilterTau,
			    				 const CONFIG &config);

		void init(void);
		void update(void);
		float getCurrent(void);
		float getVoltage(void);
		float getPower(void);
		
	protected:
		const CONFIG config_;
		const int32_t pinCurrent_;
		const int32_t pinVoltage_;
		const float currentFilterTau_;
		const float voltageFilterTau_;
		const bool filterCurrent_;
		const bool filterVoltage_;

		float currentRaw_;
		float currentFiltered_;
		float voltageRaw_;
		float voltageFiltered_;
		uint32_t tnm1_;
	};

}
#endif