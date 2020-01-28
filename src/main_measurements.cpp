// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <chrono>
#include <thread>
#include "linux_memory_process_measurement.hpp"
#include "linux_cpu_process_measurement.hpp"
#include "linux_memory_system_measurement.hpp"
#include "linux_cpu_system_measurement.hpp"

void show_usage()
{
    std::cerr << "Usage: system_metric_collector [OPTION [ARG]] ...\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t--timeout arg (=60)\tTest duration\n"
              << "\t--log arg (=out.csv)\tLog filename\n"
              << "\t--process_pid arg\tProcess PID\n"
              << std::endl;
}

int main(int argc, char * argv[])
{
  float timeout = 60;
  std::ofstream m_os;
  std::string process_pid;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-h") || (arg == "--help")) {
        show_usage();
        return 0;
    } else if (arg == "--process_pid") {
        if (i + 1 < argc) { // Make sure we aren't at the end of argv!
            process_pid = argv[++i]; // Increment 'i' so we don't get the argument as the next argv[i].
        } else { // Uh-oh, there was no argument to the destination option.
            std::cerr << "--process_pid option requires one argument." << std::endl;
            return 1;
        }
    } else if (arg == "--timeout") {
        if (i + 1 < argc) { // Make sure we aren't at the end of argv!
          timeout = std::atoi(argv[++i]); // Increment 'i' so we don't get the argument as the next argv[i].
        } else { // Uh-oh, there was no argument to the destination option.
            std::cerr << "--timeout option requires one argument." << std::endl;
            return 1;
        }
    } else if (arg == "--log") {
        if (i + 1 < argc) { // Make sure we aren't at the end of argv!
          std::string filename = argv[++i];
          m_os.open(filename, std::ofstream::out);
          std::cout << "file_name " << filename << std::endl;
          if (m_os.is_open()) {
            m_os << "T_experiment" << ",cpu_usage (%)" << ",virtual memory (Mb)" <<
              ",physical memory (Mb)" << ",resident anonymous memory (Mb)" <<
              ",system_cpu_usage (%)" << ",system virtual memory (Mb)" << std::endl;
          }
        } else { // Uh-oh, there was no argument to the destination option.
            std::cerr << "--log option requires one argument." << std::endl;
            return 1;
        }
    }
  }

  auto start = std::chrono::high_resolution_clock::now();

  LinuxMemoryProcessMeasurement linux_memory_process_measurement(process_pid);
  LinuxCPUProcessMeasurement linux_cpu_process_measurement(process_pid);

  LinuxCPUSystemMeasurement linux_cpu_system_measurement;
  LinuxMemorySystemMeasurement linux_memory_system_measurement;

  while (true) {
    linux_memory_process_measurement.makeReading();

    double cpu_process_percentage =
      linux_cpu_process_measurement.getCPUCurrentlyUsedByCurrentProcess();
    double virtual_mem_process_usage = linux_memory_process_measurement.getVirtualMemUsedProcess();
    double phy_mem_process_usage = linux_memory_process_measurement.getPhysMemUsedProcess();
    double resident_anonymousMemory_process_usage =
      linux_memory_process_measurement.getResidentAnonymousMemory();
    double cpu_system_percentage = linux_cpu_system_measurement.getCPUSystemCurrentlyUsed();
    double phy_mem_system_usage = linux_memory_system_measurement.getTotalMemorySystem() -
      linux_memory_system_measurement.getAvailableMemorySystem();

    std::cout << "virtualMemUsed Process: " << virtual_mem_process_usage << " Mb" << std::endl;
    std::cout << "ResidentAnonymousMemory Process: " << resident_anonymousMemory_process_usage <<
      " Mb" <<
      std::endl;
    std::cout << "PhysMemUsedProcess: " << phy_mem_process_usage << " Mb" << std::endl;
    std::cout << "CPU Usage Process: " << cpu_process_percentage << " (%)" << std::endl;
    std::cout << "CPU Usage system: " << cpu_system_percentage << " (%)" << std::endl;
    std::cout << "PhysMemUsedsystem: " << phy_mem_system_usage << " (Mb)" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    float seconds_running = elapsed.count() / 1000;
    std::cout << "------------------------- " << std::endl;

    if (seconds_running > timeout) {
      break;
    }

    if (m_os.is_open()) {
      m_os << seconds_running << ", " << cpu_process_percentage << ", " <<
        virtual_mem_process_usage << ", " << phy_mem_process_usage << ", " <<
        resident_anonymousMemory_process_usage << ", " << cpu_system_percentage <<
        ", " << phy_mem_system_usage << std::endl;
    }
  }
}
