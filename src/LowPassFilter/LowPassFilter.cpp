#include "LowPassFilter.h"

namespace Cyberpod
{
	// Default Constructor
	LowPassFilter::LowPassFilter(const float &tau,
	                             const float &tauDer,
	                             const float &dtMin):
	tNm1_(-1.),
	tau_(tau),
	tauDer_(tauDer),
	dtMin_(dtMin),
	data_()
	{
		if(tau_<dtMin_)
			tau_ = dtMin_;

		if(tauDer<dtMin_)
			tauDer_ = dtMin_;
	}

	// Update the time and value vectors
	void LowPassFilter::update(const float &new_time_float,
	                           const float &new_val)
	{
		if(tNm1_<0.)
		{
			data_.yRaw = new_val;
			tNm1_ = new_time_float;
		}
		else
		{
			const float dtNow = new_time_float-tNm1_;
			if(dtNow>dtMin_)
			{
				data_.yDotRaw = (data_.yRaw - new_val)/dtNow;
				data_.yRaw = new_val;
				tNm1_ = new_time_float;

				if(dtNow>tau_)
					data_.y = data_.yRaw;
				else
					data_.y += dtNow*(data_.yRaw-data_.y)/tau_;

				if(dtNow>tauDer_)
					data_.yDot = data_.yDotRaw;
				else
					data_.yDot += dtNow*(data_.yDotRaw-data_.yDot)/tauDer_;
			}
		}
	}
}