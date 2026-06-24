#pragma once
#ifndef VECTOR2_HPP
#define VECTOR2_HPP

#include <cmath>

namespace sim
{
	struct vector2
	{
		double x{ 0.0 };
		double z{ 0.0 };

		vector2() = default;
		vector2(double x_, double z_) : x(x_), z(z_) { }

		vector2 operator+(const vector2& other) const
		{
			return { x + other.x, z + other.z };
		}

		vector2 operator-(const vector2& other) const
		{
			return { x - other.x, z - other.z };
		}

		vector2 operator*(double scalar) const
		{
			return { x * scalar, z * scalar };
		}

		vector2& operator+=(const vector2& other)
		{
			x += other.x;
			z += other.z;
			return *this;
		}

		double norm() const
		{
			return std::sqrt(x*x + z*z);
		}
	};
}

#endif //VECTOR2_HPP