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
#ifndef LINUX_CPU_MEASUREMENT_HPP_
#define LINUX_CPU_MEASUREMENT_HPP_

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
  explicit LinuxCPUMeasurement(std::string pid);
  double getCPUCurrentlyUsedByCurrentProcess();
  int gettimesinceboot();
  double getUptime();

private:
  void initCPUProcess();

  int numProcessors_;
  std::string pid_;

  int16_t tickspersec_;
  double cpu_usage_;
};

#endif  // LINUX_CPU_MEASUREMENT_HPP_
