#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include "State.h"

namespace sim
{
	template <typename System>
	State rk4_step(const System& system, const State& state, double t, double dt)
	{
		const StateDerivative k1 = system(t, state);
		const StateDerivative k2 = system(t + 0.5 * dt, state + (k1 * (0.5 * dt)));
		const StateDerivative k3 = system(t + 0.5 * dt, state + (k2 * (0.5 * dt)));
		const StateDerivative k4 = system(t + dt, state + (k3 * dt));

		StateDerivative total =
			(k1 * (1.0 / 6.0)) +
			(k2 * (1.0 / 3.0)) +
			(k3 * (1.0 / 3.0)) +
			(k4 * (1.0 / 6.0));

		return state + (total * dt);

	}
}

#endif //INTEGRATOR_HPP