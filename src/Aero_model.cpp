#include "Aero_model.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace sim
{
	AeroForcesMoments Aero_model::evaluate(const Vehicle& vehicle, const State& state, const ControlInput& control, double rho) const
	{
		AeroForcesMoments afm{};

		constexpr double min_speed = 1.0;
		constexpr double alpha_limit = 0.35;
		const double V = std::max(state.V, min_speed);

		//Aero parameters
		afm.alpha = state.theta - state.gamma;
		afm.alpha = std::clamp(afm.alpha, -0.35, 0.35);
		afm.dynamic_pressure = 0.5 * rho * std::pow(V, 2);

		//Coefficients
		const double cl = vehicle.cl_alpha * afm.alpha + vehicle.cl_delta * control.elevator_deflection;
		const double cd = vehicle.cd0 + vehicle.cd_k * std::pow(cl, 2);
		const double cm = vehicle.cm_alpha * afm.alpha + vehicle.cm_q * (state.q * vehicle.reference_length / (2.0 * V)) + vehicle.cm_delta * control.elevator_deflection;

		//Aero forces and pitching moment
		afm.lift = afm.dynamic_pressure * vehicle.reference_area * cl;
		afm.drag = afm.dynamic_pressure * vehicle.reference_area * cd;
		afm.pitch_moment = afm.dynamic_pressure * vehicle.reference_area * vehicle.reference_length * cm;

		return afm;
	}
}
