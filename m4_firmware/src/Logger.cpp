#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include "utils.h"
#include "Logger.h"

Logger::Logger(const std::string& logname, const std::vector<std::string>& column_names,  const std::string& location)
		: column_names(column_names), logname(logname), location(location)
{
		logpath = location + "/" + std::string(getCurrentTimeAsString());
		if(!is_directory(logpath.c_str()))
			create_directory(logpath);

		// Write column names as header
		std::ofstream logfile(logpath + "/" + logname);
		logfile << "timestamp";
		for (int i=0; i < column_names.size(); i++)
		{
				logfile << "," << column_names[i];
		}

		logfile << "\n";
		printf("\033[0m[INFO] New Log Created: %s\n", (logpath + "/" + logname).c_str());
}

void Logger::new_log() {
	logpath = location + "/" + std::string(getCurrentTimeAsString());
	create_directory(logpath);
}

void Logger::create_directory(const std::string& path)
			{
				int result = mkdir(path.c_str(), 0777); // Use the specified mode for permissions

				if (result != 0)
				{
					if (errno == ENOENT) {
						// Create parent directories recursively
						size_t pos = path.find_last_of('/');
						if (pos != std::string::npos) {
								create_directory(path.substr(0, pos));
								result = mkdir(path.c_str(), 0777); // Try creating the directory again
						}
					}

					if (result != 0) {
						throw std::runtime_error("\033[1;31m[ERROR] Error creating directory: " + path);
					}
				}
			}
