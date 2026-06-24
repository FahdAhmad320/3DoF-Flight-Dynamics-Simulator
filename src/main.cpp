#include <iostream>
#include <exception>
#include "simulator.h"
#include "csv_writer.h"

int main()
{
	try
	{
		sim::Vehicle vehicle{};
		vehicle.mass = 120.0;
		vehicle.inertia_yy = 18.0;
		vehicle.reference_area = 0.75;      // Increased lifting surface area for realistic wing loading
		vehicle.reference_length = 1.5;     // Cleaned geometric dimension scale
		vehicle.cl_alpha = 4.2;
		vehicle.cl_delta = 0.25;
		vehicle.cd0 = 0.04;                 // Parasitic drag coefficient for a sleek aerodynamic profile
		vehicle.cd_k = 0.15;                // Reduced induced drag factor
		vehicle.cm_alpha = -1.2;
		vehicle.cm_q = -8.0;
		vehicle.cm_delta = -0.9;
		vehicle.thrust = 2500.0;

		sim::Atmosphere atmosphere;
		sim::Aero_model aero_model;
		sim::Simulator simulator(vehicle, atmosphere, aero_model);

		sim::State initial_state{};
		initial_state.x = 0.0;
		initial_state.z = 1000.0;
		initial_state.V = 86.0;             // Elevated initial velocity to match local aerodynamic trim constraints
		initial_state.gamma = 0.0;          // Horizontal launch flight path angle
		initial_state.theta = 2.0 * 3.14159265358979323846 / 180.0; // Moderate pitch initialization
		initial_state.q = 0.0;

		const double t_end = 300.0;
		const double dt = 0.01;

		const auto history = simulator.run(initial_state, t_end, dt);

		sim::csv_writer writer;
		writer.write("data/trajectory.csv", history);

		const auto& final = history.back();

		std::cout << "Simulation complete.\n";
		std::cout << "Final range: " << final.state.x << "m\n";
		std::cout << "Final altitude: " << final.state.z << "m\n";
		std::cout << "Final speed: " << final.state.V << "m/s\n";
		std::cout << "CSV written to data/trajectory.csv\n";
	}
	catch (const std::exception& ex)
	{
		std::cerr << "Error: " << ex.what() << '\n';
		return 1;
	}
	return 0;
}