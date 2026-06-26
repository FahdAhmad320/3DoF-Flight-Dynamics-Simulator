#include "csv_writer.h"

#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <filesystem>

namespace sim
{
	void csv_writer::write(const std::string& file_path, const std::vector<Snapshot>& history) const
	{
		const std::filesystem::path path(file_path);

		if (path.has_parent_path())
		{
			std::filesystem::create_directories(path.parent_path());
		}


		std::ofstream file(file_path);
		if (!file.is_open())
		{
			throw std::runtime_error("Failed to open CSV file: " + file_path);
		}

		file << "time (s),x (m),z (m),V (m/s),gamma_Deg (deg),theta_deg (deg),q_deg_s (deg/s),alpha_deg (deg),lift (N),drag (N),pitch_moment (N.m),density (kg/m^3), L/D ratio\n";
		file << std::fixed << std::setprecision(6);

		constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;

		for (const auto& s : history)
		{ 
			file << s.time << ","
				<< s.state.x << ","
				<< s.state.z << ","
				<< s.state.V << ","
				<< s.state.gamma * rad_to_deg << ","
				<< s.state.theta * rad_to_deg << ","
				<< s.state.q * rad_to_deg << ","
				<< s.alpha * rad_to_deg << ","
				<< s.lift << ","
				<< s.drag << ","
				<< s.pitch_moment << ","
				<< s.density << ","
				<< s.lift_to_drag_ratio << "\n";
		}
	}
}




