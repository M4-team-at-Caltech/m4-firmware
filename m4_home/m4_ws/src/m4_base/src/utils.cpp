#include <chrono>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <algorithm>
#include "ConsoleFormatting.h"
#include "utils.h"

double get_duration_secs(std::chrono::time_point<std::chrono::high_resolution_clock> t0, std::chrono::time_point<std::chrono::high_resolution_clock> t1)
{
	std::chrono::duration<double> duration_secs = t1 - t0;
	double dt = duration_secs.count();
	return dt;
}

bool is_directory(const char* path)
{
	struct stat info;

	if(stat(path, &info) != 0)
		printf( "%s[ERROR] Cannot access %s\n", CONSOLE_MSG_ERROR, path);
	else if(info.st_mode & S_IFDIR)
		return true;
	
	return false;
}

bool IsSubset(std::vector<int> superset, std::vector<int> subset)
/* Helper function used to identify if provided motor ids are contained within connected motor ids*/
{
    std::sort(superset.begin(), superset.end());
    std::sort(subset.begin(), subset.end());
    return std::includes(superset.begin(), superset.end(), subset.begin(), subset.end());
}

int twos_complement(int num)
{
	// Get the number of bytes of the integer (assuming it's either 2 or 4 bytes)
    int numBytes = sizeof(num);

    // Mask to isolate the most significant bit for the given number of bytes
    int msbMask = 1 << ((numBytes * 8) - 1);

    // Check if the number is negative (using the most significant bit)
    bool isNegative = (num & msbMask) != 0;

    // Take the 2's complement only for negative numbers
    if (isNegative) 
	{
        num = ~num; // Invert all the bits
        num += 1;      // Add 1
    }

    return num;
}

std::string datetime_to_str_seconds(const std::chrono::system_clock::time_point& tp)
{
    std::time_t now = std::chrono::system_clock::to_time_t(tp);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    return buffer;
}

std::string getCurrentTimeAsString() {
    std::time_t now = std::time(nullptr);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", std::localtime(&now));
    return buffer;
}