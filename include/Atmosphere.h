#ifndef ATMOSPHERE_HPP
#define ATMOSPHERE_HPP

namespace sim
{
	struct AtmosphereSample
	{
		double temperature{ 288.15 }; //k
		double pressure{ 101325.0 }; //Pa
		double density{ 1.225 };	//kg/m^3
	};

	class Atmosphere
	{
	public:
		AtmosphereSample sample (double altitude_m) const;
	};
}

#endif //ATMOSPHERE_HPP
