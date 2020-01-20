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

#include <boost/program_options.hpp>
#include <string>
#include <chrono>
#include <thread>
#include "linux_memory_process_measurement.hpp"
#include "linux_cpu_process_measurement.hpp"
#include "linux_memory_system_measurement.hpp"
#include "linux_cpu_system_measurement.hpp"

int main(int argc, char * argv[])
{
  float timeout = 60;
  std::ofstream m_os;
  std::string pid_sub;
  std::string process_name;
  std::string process_arguments;

  try {
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("help,h", "Help screen")("timeout",
      boost::program_options::value<float>()->default_value(30), "Test duration")("log",
      boost::program_options::value<std::string>()->default_value("out.csv"),
      "Log filename")("process_name",
      boost::program_options::value<std::string>()->default_value(""),
      "process_name")("process_arguments",
      boost::program_options::value<std::string>()->default_value(""), "process_arguments");

    boost::program_options::variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      std::cout << desc << '\n';
      return 0;
    }
    if (vm.count("timeout")) {
      timeout = vm["timeout"].as<float>();
      printf("Duration of the test %5f\n", timeout);
    }

    if (vm.count("log")) {
      m_os.open(vm["log"].as<std::string>(), std::ofstream::out);
      std::cout << "file_name " << vm["log"].as<std::string>() << std::endl;
      if (m_os.is_open()) {
        m_os << "T_experiment" << ",cpu_usage (%)" << ",virtual memory (Mb)" <<
          ",physical memory (Mb)" << ",resident anonymous memory (Mb)" <<
          ",system_cpu_usage (%)" << ",system virtual memory (Mb)" << std::endl;
      }
    }

    if (vm.count("process_name")) {
      process_name = vm["process_name"].as<std::string>();
      std::cout << "process_name " << process_name << std::endl;
    }
    if (vm.count("process_arguments")) {
      process_arguments = vm["process_arguments"].as<std::string>();
      std::cout << "process_arguments " << process_arguments << std::endl;
    }
  } catch (const boost::program_options::error & ex) {
    std::cerr << ex.what() << '\n';
  }

  auto start = std::chrono::high_resolution_clock::now();

  while (true) {
    pid_sub = getPIDByName(process_name, argv[0], process_arguments);
    std::cout << "pid_sub " << pid_sub << std::endl;

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;
    float seconds_running = elapsed.count() / 1000;

    if (seconds_running > timeout) {
      return -1;
    }

    if (pid_sub.empty() || atoi(pid_sub.c_str()) < 1024) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    } else {
      break;
    }
  }

  LinuxMemoryProcessMeasurement linux_memory_process_measurement(pid_sub);
  LinuxCPUProcessMeasurement linux_cpu_process_measurement(pid_sub);

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
