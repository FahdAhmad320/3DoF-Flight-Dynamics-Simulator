#include "Atmosphere.h"
#include <algorithm>
#include <cmath>

namespace sim
{
	AtmosphereSample Atmosphere::sample (double altitude_m) const
	{
		constexpr double T0 = 288.15;	//K
		constexpr double P0 = 101325.0;	//Pa
		constexpr double L = 0.0065;	//k/m
		constexpr double R = 287.05;	//J/(kg K)
		constexpr double g = 9.80665;	//m/s^2

		altitude_m = std::max(0.0, altitude_m);

		AtmosphereSample a{};

		if (altitude_m <= 11000.0)
		{
			a.temperature = T0 - L * altitude_m;
			a.pressure = P0 * std::pow(a.temperature / T0, g / (R * L));
			a.density = a.pressure / (R * a.temperature);
		}
		else
		{
			a.temperature = 216.65;
			a.pressure = 22632.1 * std::exp(-g * (altitude_m - 11000) / (R * a.temperature));
			a.density = a.pressure / (R * a.temperature);
		}

		return a;
	}
}


