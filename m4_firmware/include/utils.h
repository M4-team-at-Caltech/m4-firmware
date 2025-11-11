#ifndef UTILS_H
#define UTILS_H 

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

// Helper Functions
bool IsSubset(std::vector<int> superset, std::vector<int> subset);
double get_duration_secs(std::chrono::time_point<std::chrono::high_resolution_clock> t0, std::chrono::time_point<std::chrono::high_resolution_clock> t1);
int twos_complement(int num, int numBytes);
bool is_directory(const char* path);
std::string datetime_to_str_seconds(const std::chrono::system_clock::time_point& tp);
std::string getCurrentTimeAsString();

template<typename T>
inline std::string vector_to_string(std::vector<T> vec)
{
	std::stringstream ss;
	ss << "[";
	for(int i = 0; i < vec.size(); i++)
	{
		ss << vec[i];
		if(i!= vec.size() - 1)
			ss << ", ";
	}
	ss << "]";
	return ss.str();
}

template <typename T>
class CircularBuffer {
public:
    explicit CircularBuffer(std::size_t size) 
        : data(size), capacity(size), head(0), tail(0) {}

    void push(const T& item) {
        data[head] = item;
        head = (head + 1) % capacity;
    }

    T pop() {
        T item = data[tail];
        tail = (tail + 1) % capacity;
        return item;
    }

	bool empty() {
		return head == tail;
    }

private:
    std::vector<T> data;
    std::size_t capacity;
    std::size_t head;
    std::size_t tail;
};
#endif