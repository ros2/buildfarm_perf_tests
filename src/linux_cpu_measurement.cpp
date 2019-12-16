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
#include "linux_cpu_measurement.hpp"
#include <string>
#include <vector>

LinuxCPUMeasurement::LinuxCPUMeasurement(std::string pid)
: pid_(pid),
  tickspersec_(sysconf(_SC_CLK_TCK)),
  numProcessors_(0)
{
  initCPUProcess();
}

void LinuxCPUMeasurement::initCPUProcess()
{
  std::string line;
  std::ifstream myfile(std::string("/proc/cpuinfo"));
  if (myfile.is_open()) {
    while (std::getline(myfile, line) ) {
      if (line.find("processor") != std::string::npos) {
        this->numProcessors_++;
      }
    }
  }
  myfile.close();
}

int LinuxCPUMeasurement::gettimesinceboot()
{
  double result = 0;
  std::string line;
  std::ifstream myfile(std::string("/proc/uptime"));
  if (myfile.is_open()) {
    while (std::getline(myfile, line) ) {
      std::vector<std::string> tokens = split(line, ' ');
      result = atof(tokens[0].c_str()) * tickspersec_ + atof(tokens[1].c_str());
    }
  }
  myfile.close();
  return result;
}

double LinuxCPUMeasurement::getUptime()
{
  double uptime = 0;
  std::string line;
  std::ifstream myfile(std::string("/proc/uptime"));
  if (myfile.is_open()) {
    while (std::getline(myfile, line) ) {
      std::vector<std::string> tokens = split(line, ' ');
      uptime = atof(tokens[0].c_str());
    }
  }
  myfile.close();
  return uptime;
}

double LinuxCPUMeasurement::getCPUCurrentlyUsedByCurrentProcess()
{
  int64_t pid;
  std::string tcomm_string;
  char state;

  int64_t ppid;
  int64_t pgid;
  int64_t sid;
  int64_t tty_nr;
  int64_t tty_pgrp;

  int64_t flags;
  int64_t min_flt;
  int64_t cmin_flt;
  int64_t maj_flt;
  int64_t cmaj_flt;
  int64_t utime;
  int64_t stimev;

  int64_t cutime;
  int64_t cstime;
  int64_t priority;
  int64_t nicev;
  int64_t num_threads;
  int64_t it_real_value;

  uint64_t start_time;

  int64_t vsize;
  int64_t rss;
  int64_t rsslim;
  int64_t start_code;
  int64_t end_code;
  int64_t start_stack;
  int64_t esp;
  int64_t eip;

  int64_t pending;
  int64_t blocked;
  int64_t sigign;
  int64_t sigcatch;
  int64_t wchan;
  int64_t zero1;
  int64_t zero2;
  int64_t exit_signal;
  int64_t cpu;
  int64_t rt_priority;
  int64_t policy;

  double uptime = getUptime();

  std::ifstream inputFile(std::string("/proc/") + this->pid_ + std::string("/stat"));

  inputFile >> pid >> tcomm_string >> state >> ppid >> pgid >> sid >> tty_nr >>
  tty_pgrp >> flags >> min_flt >> cmin_flt >> maj_flt >> cmaj_flt >>
  utime >> stimev >> cutime >> cstime >> priority >> nicev >> num_threads >>
  it_real_value >> start_time >> vsize >> rss >> rsslim >> start_code >>
  end_code >> start_stack >> esp >> eip >> pending >> blocked >> sigign >>
  sigcatch >> wchan >> zero1 >> zero2 >> exit_signal >> cpu >> rt_priority >>
  policy;

  double total_time = static_cast<double>(utime) + static_cast<double>(stimev);
  total_time += static_cast<double>(cstime) + static_cast<double>(cutime);

  double seconds = uptime - (start_time / this->tickspersec_);

  this->cpu_usage_ = 100 * ((total_time / this->tickspersec_) / seconds) / this->numProcessors_;

  return this->cpu_usage_;
}
