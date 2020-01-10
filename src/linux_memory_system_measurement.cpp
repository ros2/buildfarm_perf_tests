#include "linux_memory_system_measurement.hpp"

LinuxMemorySystemMeasurement::LinuxMemorySystemMeasurement()
{

}

double LinuxMemorySystemMeasurement::getTotalMemorySystem() {
  std::string token;
  std::ifstream file("/proc/meminfo");
  while(file >> token) {
      if(token == "MemTotal:") {
          unsigned long mem;
          if(file >> mem) {
              return mem / 1024.0; // Mb
          } else {
              return 0;
          }
      }
      // ignore rest of the line
      file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return 0;
}

double LinuxMemorySystemMeasurement::getFreeMemorySystem() {
  std::string token;
  std::ifstream file("/proc/meminfo");
  while(file >> token) {
      if(token == "MemFree:") {
          unsigned long mem;
          if(file >> mem) {
              return mem / 1024.0; // Mb
          } else {
              return 0;
          }
      }
      // ignore rest of the line
      file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return 0;
}

double LinuxMemorySystemMeasurement::getAvailableMemorySystem() {
  std::string token;
  std::ifstream file("/proc/meminfo");
  while(file >> token) {
      if(token == "MemAvailable:") {
          unsigned long mem;
          if(file >> mem) {
              return mem / 1024.0; // Mb
          } else {
              return 0;
          }
      }
      // ignore rest of the line
      file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  return 0;
}
