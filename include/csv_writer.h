#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <string>
#include <vector>
#include "simulator.h"

namespace sim
{
	class csv_writer
	{
	public:
		void write(const std::string& file_path, const std::vector<Snapshot>& history) const;
	};
}

#endif //CSV_WRITER_H