#ifndef LINUX_CPU_SYSTEM_MEASUREMENT_HPP
#define LINUX_CPU_SYSTEM_MEASUREMENT_HPP

#include <unistd.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <iostream>

enum CPUStates
{
  S_USER = 0,
    S_NICE,
    S_SYSTEM,
    S_IDLE,
    S_IOWAIT,
    S_IRQ,
    S_SOFTIRQ,
    S_STEAL,
    S_GUEST,
    S_GUEST_NICE,
};

const int NUM_CPU_STATES = 10;

typedef struct CPUData
{
	std::string cpu;
	size_t times[NUM_CPU_STATES];
} CPUData;

class LinuxCPUSystemMeasurement
{
public:
    explicit LinuxCPUSystemMeasurement();
    void ReadStatsCPU(std::vector<CPUData> & entries);
    float getCPUSystemCurrentlyUsed();

private:
  size_t GetIdleTime(const CPUData & e);
  size_t GetActiveTime(const CPUData & e);
  void initCPUProcess();
  void makeReading();

  std::vector<CPUData> entries1_;
  std::vector<CPUData> entries2_;
  int numProcessors_;

};

#endif // LINUX_CPU_SYSTEM_MEASUREMENT_HPP
