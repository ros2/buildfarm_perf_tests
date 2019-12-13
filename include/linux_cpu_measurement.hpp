#ifndef LINUX_CPU_MEASUREMENT_HPP
#define LINUX_CPU_MEASUREMENT_HPP

#include <unistd.h>
#include <time.h>
#include <linux/limits.h>
#include <sys/times.h>


#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "utilities/utilities.hpp"

class LinuxCPUMeasurement
{
public:
    LinuxCPUMeasurement(std::string pid);
    double getCPUCurrentlyUsedByCurrentProcess();
    int gettimesinceboot();
    double getUptime();

private:

  void initCPUProcess();

  int numProcessors_;
  std::string pid_;

  long tickspersec_;
  double cpu_usage_;
};

#endif // LINUX_MEMORY_MEASUREMENT_HPP
