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
#ifndef LINUX_MEMORY_PROCESS_MEASUREMENT_HPP_
#define LINUX_MEMORY_PROCESS_MEASUREMENT_HPP_

#include <sys/types.h>
#include <sys/sysinfo.h>
#include <stdlib.h>
#include <stdio.h>

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "utilities/utilities.hpp"

class LinuxMemoryProcessMeasurement
{
public:
  explicit LinuxMemoryProcessMeasurement(std::string pid);
  void makeReading();

  inline double getTotalVirtualMem() {return this->totalVirtualMem_;}
  inline double getVirtualMemUsed() {return this->virtualMemUsed_;}
  inline double getVirtualMemUsedProcess() {return this->virtualMemUsedProcess_;}
  inline double getTotalPhysMem() {return this->totalPhysMem_;}
  inline double getPhysMemUsed() {return this->physMemUsed_;}
  inline double getPhysMemUsedProcess() {return this->physMemUsedProcess_;}
  inline double getResidentAnonymousMemory() {return this->residentAnonymousMemory_;}

private:
  int getResidentAnonymousMemoryCurrentlyUsedByCurrentProcess();
  int getVirtualMemoryCurrentlyUsedByCurrentProcess();
  int getPhysicalMemoryCurrentlyUsedByCurrentProcess();

  double totalVirtualMem_;
  double virtualMemUsed_;
  double residentAnonymousMemory_;
  double virtualMemUsedProcess_;
  double totalPhysMem_;
  double physMemUsed_;
  double physMemUsedProcess_;
  std::string pid_;
};

#endif  // LINUX_MEMORY_PROCESS_MEASUREMENT_HPP_
