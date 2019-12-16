#include "linux_memory_measurement.hpp"

LinuxMemoryMeasurement::LinuxMemoryMeasurement(std::string pid):
  totalVirtualMem_(0),
  virtualMemUsed_(0),
  virtualMemUsedProcess_(0),
  totalPhysMem_(0),
  physMemUsed_(0),
  physMemUsedProcess_(0),
  pid_(pid)
{
}

int LinuxMemoryMeasurement::getVirtualMemoryCurrentlyUsedByCurrentProcess(){ //Note: this value is in KB!
  std::string line;
  std::ifstream myfile (std::string("/proc/") + this->pid_.c_str() + std::string("/status"));
  if (myfile.is_open()) {
    while ( std::getline (myfile,line) ) {
      if (line.find("VmSize:") != std::string::npos) {
        std::vector<std::string> tokens = split(line, ' ');
        myfile.close();
        return atoi(tokens[1].c_str());
      }
    }
  }
  return -1;
}

int LinuxMemoryMeasurement::getResidentAnonymousMemoryCurrentlyUsedByCurrentProcess(){ //Note: this value is in KB!
  std::string line;
  std::ifstream myfile (std::string("/proc/") + this->pid_.c_str() + std::string("/status"));
  if (myfile.is_open()) {
    while ( std::getline (myfile,line) ) {
      std::cerr << line << std::endl;
      if (line.find("RssAnon:") != std::string::npos) {
        std::vector<std::string> tokens = split(line, ' ');
        myfile.close();
        return atoi(tokens[1].c_str());
      }
    }
  }
  return -1;
}

int LinuxMemoryMeasurement::getPhysicalMemoryCurrentlyUsedByCurrentProcess(){ //Note: this value is in KB!
  std::string line;
  std::ifstream myfile (std::string("/proc/") + this->pid_.c_str() + std::string("/status"));
  if (myfile.is_open()) {
    while ( std::getline (myfile,line) ) {
      if (line.find("VmRSS:") != std::string::npos) {
        std::vector<std::string> tokens = split(line, ' ');
        myfile.close();
        return atoi(tokens[1].c_str());
      }
    }
  }
  return -1;
}

void LinuxMemoryMeasurement::makeReading()
{
  struct sysinfo memInfo;
  sysinfo (&memInfo);
  long long totalVirtualMem = memInfo.totalram;
  //Add other values in next statement to avoid int overflow on right hand side...
  totalVirtualMem += memInfo.totalswap;
  totalVirtualMem *= memInfo.mem_unit;
  this->totalVirtualMem_ = totalVirtualMem/1024.0/1024.0; //Mb

  long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
  //Add other values in next statement to avoid int overflow on right hand side...
  virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
  virtualMemUsed *= memInfo.mem_unit;
  this->virtualMemUsed_ = virtualMemUsed/1024.0/1024.0; //Mb

  this->virtualMemUsedProcess_ = getVirtualMemoryCurrentlyUsedByCurrentProcess()/1024.0; //Mb

  this->residentAnonymousMemory_ = getResidentAnonymousMemoryCurrentlyUsedByCurrentProcess()/1024.0; //Mb

  long long  totalPhysMem = memInfo.totalram;
  //Multiply in next statement to avoid int overflow on right hand side...
  totalPhysMem *= memInfo.mem_unit;
  this->totalPhysMem_ = totalPhysMem/1024.0/1024.0; //Mb

  long long  physMemUsed = memInfo.totalram - memInfo.freeram;
  physMemUsed *= memInfo.mem_unit;
  this->physMemUsed_ = physMemUsed/1024.0/1024.0;

  this->physMemUsedProcess_ = getPhysicalMemoryCurrentlyUsedByCurrentProcess()/1024.0; //Mb

}
