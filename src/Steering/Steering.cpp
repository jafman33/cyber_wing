#include "Steering.h"

namespace Cyberpod
{

	Steering::Steering(const int32_t &pin,
			               const float &filterTau):
	config_(),
	pin_(pin),
	filterTau_(filterTau),
	data_(),
	tnm1_(0),
	firstReceived_(false)
	{}


	void Steering::init(void)
	{
		analogReadResolution(16);
		tnm1_ = micros();
		pinMode(pin_,INPUT);
	}

	void Steering::update(void)
	{
		uint32_t t = micros();
		float dt = static_cast<float>(t - tnm1_)/1000000.0F;
		tnm1_ = t;

		//Current
		data_.raw = config_.slope*(static_cast<float>(analogRead(pin_))/65535.0F - config_.intercept);

		if(firstReceived_)
		{
			data_.filtered += dt*(data_.raw - data_.filtered)/filterTau_;
		}

		firstReceived_ = true;
	}
	
}