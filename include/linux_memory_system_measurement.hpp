#ifndef LINUX_MEMORY_SYSTEM_MEASUREMENT_HPP
#define LINUX_MEMORY_SYSTEM_MEASUREMENT_HPP

#include <unistd.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <limits>

class LinuxMemorySystemMeasurement
{
public:
    LinuxMemorySystemMeasurement();
    double getTotalMemorySystem();
    double getFreeMemorySystem();
    double getAvailableMemorySystem();
};

#endif // LINUX_MEMORY_SYSTEM_MEASUREMENT_HPP
