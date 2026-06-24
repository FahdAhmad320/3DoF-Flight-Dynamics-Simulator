#ifndef AERO_MODEL_HPP
#define AERO_MODEL_HPP

#include "State.h"
#include "Vehicle.h"

namespace sim
{
	struct AeroForcesMoments
	{
		double lift{ 0.0 };
		double drag{ 0.0 };
		double pitch_moment{ 0.0 };
		double alpha{ 0.0 };
		double dynamic_pressure{ 0.0 };
	};

	class Aero_model
	{
	public:
		AeroForcesMoments evaluate (
			const Vehicle& vehicle, 
			const State& state, 
			const ControlInput& control, 
			double rho) const;		
	};
}

#endif //AERO_MODEL_HPP
