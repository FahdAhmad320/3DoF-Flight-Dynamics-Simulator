#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "Vehicle.h"
#include "Aero_model.h"
#include "Atmosphere.h"
#include "State.h"

#include <vector>

namespace sim
{
	struct Snapshot
	{
		double time{ 0.0 };
		State state{};
		double alpha{ 0.0 };
		double lift{ 0.0 };
		double drag{ 0.0 };
		double pitch_moment{ 0.0 };
		double density{ 0.0 };
	};

	class Simulator
	{
	private:
		Vehicle vehicle_;
		Atmosphere atmosphere_;
		Aero_model aero_model_;
		ControlInput control_law(double t, const State& state) const;
		StateDerivative derivatives(double t, const State& state) const;

	public:
		Simulator();
		Simulator(Vehicle vehicle, Atmosphere atmosphere, Aero_model aero_model);
		~Simulator();

		std::vector<Snapshot> run(const State& initial_state, double t_end, double dt) const;
	};
}

#endif //SIMULATOR_HPP
