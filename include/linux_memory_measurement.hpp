#ifndef LINUX_MEMORY_MEASUREMENT_HPP
#define LINUX_MEMORY_MEASUREMENT_HPP

#include "sys/types.h"
#include "sys/sysinfo.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include "utilities/utilities.hpp"

class LinuxMemoryMeasurement
{
public:
    explicit LinuxMemoryMeasurement(std::string pid);
    void makeReading();

    inline double getTotalVirtualMem(){ return this->totalVirtualMem_; }
    inline double getVirtualMemUsed(){ return this->virtualMemUsed_; }
    inline double getVirtualMemUsedProcess(){ return this->virtualMemUsedProcess_; }
    inline double getTotalPhysMem(){ return this->totalPhysMem_; }
    inline double getPhysMemUsed(){ return this->physMemUsed_; }
    inline double getPhysMemUsedProcess(){ return this->physMemUsedProcess_; }
    inline double getResidentAnonymousMemory(){ return this->residentAnonymousMemory_; }

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

#endif // LINUX_MEMORY_MEASUREMENT_HPP
