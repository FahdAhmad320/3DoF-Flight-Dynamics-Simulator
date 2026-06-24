#ifndef VEHICLE_HPP
#define VEHICLE_HPP

namespace sim
{
	struct Vehicle
	{
		double mass{ 120.0 };			//kg
		double inertia_yy{ 18.0 };		//kgm^2
		double reference_area{ 0.18 };	//m^2
		double reference_length{ 2.2 };	//m

		//Aerodynamic coefficients
		double cl_alpha{ 4.5 };			//1/rad
		double cl_delta{ 0.2 };			//1/rad

		double cd0{ 0.08 };
		double cd_k{ 0.65 };

		double cm_alpha{ -1.2 };		//1/rad
		double cm_q{ -8.0 };			//1/rad
		double cm_delta{ -0.8 };		//1/rad

		//Propulsion
		double thrust{ 2500.0 };		//N
	};

	struct ControlInput
	{
		double elevator_deflection{ 0.0 };	//rad
		double thrust_command{ 1.0 };		//0..1 scale
	};
}

#endif //VEHICLE_HPP
