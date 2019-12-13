#include "linux_cpu_measurement.hpp"

LinuxCPUMeasurement::LinuxCPUMeasurement(std::string pid):
  pid_(pid),
  tickspersec_(sysconf(_SC_CLK_TCK)),
  numProcessors_(0)
{
  initCPUProcess();
}

void LinuxCPUMeasurement::initCPUProcess()
{
  std::string line;
  std::ifstream myfile (std::string("/proc/cpuinfo"));
  if (myfile.is_open()) {
    while ( std::getline (myfile,line) ) {
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
  std::ifstream myfile (std::string("/proc/uptime"));
  if (myfile.is_open()) {
    while ( std::getline (myfile,line) ) {
      std::vector<std::string> tokens = split(line, ' ');
      result = atof(tokens[0].c_str())*tickspersec_ + atof(tokens[1].c_str());
    }
  }
  myfile.close();
  return result;
}

double LinuxCPUMeasurement::getUptime()
{
  double uptime = 0;
  std::string line;
  std::ifstream myfile (std::string("/proc/uptime"));
  if (myfile.is_open()) {
    while ( std::getline (myfile,line) ) {
      std::vector<std::string> tokens = split(line, ' ');
      uptime = atof(tokens[0].c_str());
    }
  }
  myfile.close();
  return uptime;
}

double LinuxCPUMeasurement::getCPUCurrentlyUsedByCurrentProcess()
{
  long long int pid;
  std::string tcomm_string;
  char state;

  long long int ppid;
  long long int pgid;
  long long int sid;
  long long int tty_nr;
  long long int tty_pgrp;

  long long int flags;
  long long int min_flt;
  long long int cmin_flt;
  long long int maj_flt;
  long long int cmaj_flt;
  long long int utime;
  long long int stimev;

  long long int cutime;
  long long int cstime;
  long long int priority;
  long long int nicev;
  long long int num_threads;
  long long int it_real_value;

  unsigned long long start_time;

  long long int vsize;
  long long int rss;
  long long int rsslim;
  long long int start_code;
  long long int end_code;
  long long int start_stack;
  long long int esp;
  long long int eip;

  long long int pending;
  long long int blocked;
  long long int sigign;
  long long int sigcatch;
  long long int wchan;
  long long int zero1;
  long long int zero2;
  long long int exit_signal;
  long long int cpu;
  long long int rt_priority;
  long long int policy;

  double uptime = getUptime();

  std::ifstream inputFile (std::string("/proc/") + this->pid_ + std::string("/stat"));

  inputFile >> pid >> tcomm_string >> state >> ppid >> pgid >> sid >> tty_nr
            >> tty_pgrp >> flags 	>> min_flt 	>> cmin_flt >> maj_flt 	>> cmaj_flt
            >> utime 	>> stimev >> cutime >> cstime >> priority >> nicev >> num_threads
            >> it_real_value >> start_time >> vsize >> rss >> rsslim >> start_code
            >> end_code >> start_stack >> esp >> eip >> pending >> blocked >> sigign
            >> sigcatch >> wchan >> zero1 >> zero2 >> exit_signal >> cpu >> rt_priority
            >> policy;

  double total_time = (((double)utime)) + (((double)stimev));
  total_time += (((double)cstime)) + (((double)cutime));

  double seconds = uptime - (start_time/ this->tickspersec_);// / '$clock_ticks') )}' )

  this->cpu_usage_ = 100 * ((total_time / this->tickspersec_) / seconds)/this->numProcessors_;

  return this->cpu_usage_;
}
