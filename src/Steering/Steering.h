#ifndef __STEERING_H_INCLUDED__
#define __STEERING_H_INCLUDED__

#include "Arduino.h"

namespace Cyberpod
{

	class Steering
	{
	public:
		struct CONFIG
		{
			float slope = -5.0F;
			float intercept = 0.49F;
		};

		struct DATA
		{
			float raw = 0.0F;
			float filtered = 0.0F;
		};

	public:
		Steering(const int32_t &pin,
			       const float &filterTau);
	

		void init(void);
		void update(void);
		DATA data_;

	protected:
		const CONFIG config_;
		const int32_t pin_;
		const float filterTau_;
		uint32_t tnm1_;
		bool firstReceived_;
	};

}
#endif