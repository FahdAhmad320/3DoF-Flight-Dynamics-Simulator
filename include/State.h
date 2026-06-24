#ifndef STATE_HPP
#define STATE_HPP

namespace sim
{
	struct State
	{
		double x{ 0.0 };
		double z{ 0.0 };
		double V{ 0.0 };
		double gamma{ 0.0 };
		double theta{ 0.0 };
		double q{ 0.0 };
	};

	struct StateDerivative
	{
		double x_dot{ 0.0 };
		double z_dot{ 0.0 };
		double V_dot{ 0.0 };
		double gamma_dot{ 0.0 };
		double theta_dot{ 0.0 };
		double q_dot{ 0.0 };
	};

	inline State operator+(const State& s, const StateDerivative& ds)
	{
		return
		{
			s.x + ds.x_dot,
			s.z + ds.z_dot,
			s.V + ds.V_dot,
			s.gamma + ds.gamma_dot,
			s.theta + ds.theta_dot,
			s.q + ds.q_dot
		};
	}

	inline StateDerivative operator*(const StateDerivative& ds, double scalar)
	{
		return
		{
			ds.x_dot * scalar,
			ds.z_dot * scalar,
			ds.V_dot * scalar,
			ds.gamma_dot * scalar,
			ds.theta_dot * scalar,
			ds.q_dot * scalar
		};
	}

	inline StateDerivative operator+(const StateDerivative& a, const StateDerivative& b)
	{
		return
		{
			a.x_dot + b.x_dot,
			a.z_dot + b.z_dot,
			a.V_dot + b.V_dot,
			a.gamma_dot + b.gamma_dot,
			a.theta_dot + b.theta_dot,
			a.q_dot + b.q_dot
		};
	}
}

#endif //STATE_HPP