#include "simulator.h"
#include "integrator.h"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace sim
{
	namespace
	{
		constexpr double g = 9.80665;

		double calculate_lift_to_drag_ratio(double lift, double drag)
		{
			constexpr double min_drag = 1.0e-9;

			if (std::abs(drag) < min_drag)
			{
				return 0.0;
			}

			return lift / drag;
		}
	}

	// Constructors
	Simulator::Simulator() {}

	Simulator::Simulator(Vehicle vehicle, Atmosphere atmosphere, Aero_model aero_model)
		: vehicle_(vehicle), atmosphere_(atmosphere), aero_model_(aero_model) {
	}

	// Destructor
	Simulator::~Simulator() = default;

	ControlInput Simulator::control_law(double t, const State& state) const
	{
		ControlInput u{};

		constexpr double deg_to_rad = 3.14159265358979323846 / 180.0;

		double theta_cmd = 0.0 * deg_to_rad;

		if (t < 5.0)
		{
			theta_cmd = 2.0 * deg_to_rad;
		}
		else if (t < 6.0)
		{
			theta_cmd = -2.0 * deg_to_rad;
		}
		else
		{
			theta_cmd = 0.0 * deg_to_rad;
		}

		// Tuned pitch-attitude control gains.
		// Proportional term tracks commanded pitch attitude.
		// Derivative term provides pitch-rate damping.
		const double kp = 2.5;
		const double kd = 0.6;

		const double theta_error = state.theta - theta_cmd;

		// Elevator deflection command.
		// With negative Cm_delta, positive elevator command produces a restoring pitch moment.
		u.elevator_deflection = (kp * theta_error) + (kd * state.q);

		constexpr double max_deflection = 20.0 * deg_to_rad;
		u.elevator_deflection =
			std::clamp(u.elevator_deflection, -max_deflection, max_deflection);

		// Thrust command set to idle/off for a gliding descent case.
		u.thrust_command = 0.0;

		return u;
	}

	StateDerivative Simulator::derivatives(double t, const State& state) const
	{
		if (state.z < 0.0)
		{
			return {};
		}

		const ControlInput control = control_law(t, state);
		const AtmosphereSample atm = atmosphere_.sample(state.z);
		const AeroForcesMoments afm =
			aero_model_.evaluate(vehicle_, state, control, atm.density);

		const double thrust = vehicle_.thrust * control.thrust_command;
		const double mass = vehicle_.mass;

		const double x_dot = state.V * std::cos(state.gamma);
		const double z_dot = state.V * std::sin(state.gamma);

		const double V_dot =
			(thrust * std::cos(state.theta - state.gamma) - afm.drag) / mass
			- g * std::sin(state.gamma);

		double gamma_dot = 0.0;

		if (state.V > 1.0)
		{
			gamma_dot =
				(thrust * std::sin(state.theta - state.gamma) + afm.lift)
				/ (mass * state.V)
				- (g * std::cos(state.gamma) / state.V);
		}

		const double theta_dot = state.q;
		const double q_dot = afm.pitch_moment / vehicle_.inertia_yy;

		return { x_dot, z_dot, V_dot, gamma_dot, theta_dot, q_dot };
	}

	std::vector<Snapshot> Simulator::run(
		const State& initial_state,
		double t_end,
		double dt) const
	{
		if (dt <= 0.0 || t_end <= 0.0)
		{
			throw std::invalid_argument("t_end and dt must be positive.");
		}

		std::vector<Snapshot> history;
		history.reserve(static_cast<std::size_t>(t_end / dt) + 1);

		State state = initial_state;
		double t = 0.0;

		while (t <= t_end)
		{
			const ControlInput control = control_law(t, state);
			const AtmosphereSample atm = atmosphere_.sample(state.z);
			const AeroForcesMoments afm =
				aero_model_.evaluate(vehicle_, state, control, atm.density);

			const double lift_to_drag_ratio =
				calculate_lift_to_drag_ratio(afm.lift, afm.drag);

			history.push_back({
				t,
				state,
				afm.alpha,
				afm.lift,
				afm.drag,
				afm.pitch_moment,
				atm.density,
				lift_to_drag_ratio
				});

			state = rk4_step([this](double time, const State& s)
				{
					return derivatives(time, s);
				},
				state, t, dt);

			if (state.z < 0.0)
			{
				state.z = 0.0;

				// Recalculate atmosphere and aerodynamic data at the clamped ground state.
				const ControlInput final_control = control_law(t + dt, state);
				const AtmosphereSample final_atm = atmosphere_.sample(state.z);
				const AeroForcesMoments final_afm =
					aero_model_.evaluate(vehicle_, state, final_control, final_atm.density);

				const double final_lift_to_drag_ratio =
					calculate_lift_to_drag_ratio(final_afm.lift, final_afm.drag);

				history.push_back({
					t + dt,
					state,
					final_afm.alpha,
					final_afm.lift,
					final_afm.drag,
					final_afm.pitch_moment,
					final_atm.density,
					final_lift_to_drag_ratio
					});

				break;
			}

			t += dt;
		}

		return history;
	}
}

