#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include "ConsoleFormatting.h"
#include "utils.h"

#ifdef _WIN32
	#include <direct.h> // For _mkdir on Windows
	#define mkdir(path, mode) _mkdir(path)
#endif

class Logger
{
		public:
			Logger(const std::string& logname, const std::vector<std::string>& column_names,  const std::string& location = "./Logs");

			template<typename T>
			void log_entry(const std::vector<T>& values, long long timestamp = -1)
			{
				if (values.size() != column_names.size()) {
					printf("%s[ERROR]: Column names and values have different number of elements. Remember not to include time in column names list\n%s", CONSOLE_MSG_ERROR, CONSOLE_DEFAULT);
					exit(1);
				}

				if (timestamp == -1) {
						auto now = std::chrono::system_clock::now();
						timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
				}

				std::ofstream logfile(logpath + "/" + logname, std::ios_base::app);
				logfile << timestamp;
				for (int i=0; i < values.size(); i++)
				{
						logfile << "," << values[i];
				}
				logfile << "\n";
			}

			template<typename T>
			void log_batch_entry(const std::vector<std::vector<T>>& values)
			{
				if (values[0].size() != column_names.size() + 1) {
					printf("%s[ERROR]: Column names and values have different number of elements. Timestamp must be provided as first element of values for batch log entry\n%s", CONSOLE_MSG_ERROR, CONSOLE_DEFAULT);
					exit(1);
				}

				std::ofstream logfile(logpath + "/" + logname, std::ios_base::app);
				for (int i=0; i < values.size(); i++)
				{
					logfile << values[i][0];
					for(int j = 1; j < values[i].size(); j++)
						logfile << "," << values[i][j];
					logfile << "\n";
				}
			}

			

			void new_log();

		private:
			std::vector<std::string> column_names;
			std::string logname;
			std::string location;
			std::string logpath;

			void create_directory(const std::string& path);
};

#endif
