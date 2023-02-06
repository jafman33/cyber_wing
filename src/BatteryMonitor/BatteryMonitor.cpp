#include "BatteryMonitor.h"

namespace Cyberpod
{

	BatteryMonitor::BatteryMonitor(const int32_t &pinCurrent,
			                           const int32_t &pinVoltage,
			                           const float &currentFilterTau,
			                           const float &voltageFilterTau):
	BatteryMonitor(pinCurrent,pinVoltage,currentFilterTau,voltageFilterTau,CONFIG())
	{}

	BatteryMonitor::BatteryMonitor(const int32_t &pinCurrent,
			                           const int32_t &pinVoltage,
			                           const float &currentFilterTau,
			                           const float &voltageFilterTau,
			          						 		 const CONFIG &config):
	config_(config),
	pinCurrent_(pinCurrent),
	pinVoltage_(pinVoltage),
	currentFilterTau_(currentFilterTau),
	voltageFilterTau_(voltageFilterTau),
	filterCurrent_((currentFilterTau==0) ? false : true),
	filterVoltage_((voltageFilterTau_==0) ? false : true),
	currentRaw_(0.0F),
	currentFiltered_(0.0F),
	voltageRaw_(55.0F),
	voltageFiltered_(55.0F),
	tnm1_(0)
	{}


	void BatteryMonitor::init()
	{
		analogReadResolution(16);
		tnm1_ = micros();
		pinMode(pinCurrent_,INPUT);
		pinMode(pinVoltage_,INPUT);
	}

	void BatteryMonitor::update()
	{
		uint32_t t = micros();
		float dt = static_cast<float>(t - tnm1_)/1E6F;
		tnm1_ = t;

		//Current
		float raw = static_cast<float>(analogRead(pinCurrent_));
		currentRaw_ = ((config_.Vref*raw/65535.0F) - 330.0F)/config_.mV_A;

		if(filterCurrent_)
		{
			currentFiltered_ += dt*(currentRaw_ - currentFiltered_)/currentFilterTau_;
		}
		else
		{
			currentFiltered_ = currentRaw_;
		}

		//Voltage
		raw = static_cast<float>(analogRead(pinVoltage_));
		voltageRaw_ = (config_.Vref*raw/65535.0F)/config_.mV_V;

		if(filterVoltage_)
		{
			voltageFiltered_ += dt*(voltageRaw_ - voltageFiltered_)/voltageFilterTau_;
		}
		else
		{
			voltageFiltered_ = voltageRaw_;
		}
	}

	float BatteryMonitor::getCurrent()
	{
		return currentFiltered_;
	}

	float BatteryMonitor::getVoltage()
	{
		return voltageFiltered_;
	}

	float BatteryMonitor::getPower()
	{
		return voltageFiltered_*currentFiltered_;
	}
	
}