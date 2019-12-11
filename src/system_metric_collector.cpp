#include <unistd.h>
#include <chrono>
#include <sys/statvfs.h>

#include "system_metrics_collector/linux_cpu_measurement_node.hpp"
#include "system_metrics_collector/utilities.hpp"

#include <boost/program_options.hpp>
using namespace boost::program_options;

constexpr const char PROC_STAT_FILE_CPU[] = "/proc/stat";
constexpr const char PROC_STAT_FILE_MEM[] = "/proc/meminfo";

char * get_capacity(char * dev_path)
{
  unsigned long long result = 0;
  int n;
  char s_cap[50];
  char * ss_cap = "N/A";
  struct statvfs sfs;
  if (statvfs(dev_path, &sfs) != -1) {
    result = (unsigned long long)sfs.f_bsize * sfs.f_blocks;
  }
  if (result > 0) {
    double f_cap = (double)result / (1024);
    n = sprintf(s_cap, "%.2f", f_cap);
    ss_cap = strdup(s_cap);
  }
  return ss_cap;
}

char * get_free_space(char * dev_path)
{
  unsigned long long result = 0;
  int n;
  char s_cap[50];
  char * ss_cap = "N/A";
  struct statvfs sfs;

  if (statvfs(dev_path, &sfs) != -1) {
    result = (unsigned long long)sfs.f_bsize * sfs.f_bfree;
  }
  if (result > 0) {
    double f_cap = (double)result / (1024);
    n = sprintf(s_cap, "%.2f", f_cap);
    ss_cap = strdup(s_cap);
  }
  return ss_cap;
}

int main(int argc, char ** argv)
{

  float timeout = 60;
  std::ofstream m_os;

  try {
    options_description desc{"Options"};
    desc.add_options()
      ("help,h", "Help screen")
      ("timeout", value<float>()->default_value(30), "Test duration")
      ("log", value<std::string>()->default_value("out.csv"), "Log filename");

    variables_map vm;
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
        m_os << "T_experiment" << ",cpu_usage (%)" << ",virtual memory (%)" <<
          ",disk usage (Kb)" << ",free disk (Kb)" << std::endl;
      }
    }
  } catch (const error & ex) {
    std::cerr << ex.what() << '\n';
  }

  system_metrics_collector::ProcCpuData last_measurement_;
  system_metrics_collector::ProcCpuData current_measurement;

  printf("T_experiment, cpu_usage, virtual memory, disk capacity, disk usage\n");

  auto start = std::chrono::high_resolution_clock::now();

  while (1) {
    double cpu_percentage;
    double mem_usage;

    {
      std::ifstream stat_file(PROC_STAT_FILE_CPU);
      if (!stat_file.good()) {
        printf("unable to open file %s\n", PROC_STAT_FILE_CPU);
        return -1;
      }
      std::string line;
      std::getline(stat_file, line);

      if (!stat_file.good()) {
        printf("unable to get cpu line from file\n");
        return -1;
      } else {
        system_metrics_collector::ProcCpuData current_measurement =
          system_metrics_collector::processStatCpuLine(line);
        cpu_percentage = system_metrics_collector::computeCpuActivePercentage(
          last_measurement_,
          current_measurement);
        last_measurement_ = current_measurement;
      }
    }

    {
      std::ifstream stat_file(PROC_STAT_FILE_MEM);
      if (!stat_file.good()) {
        printf("not able to read the file\n");
        return -1;
      }
      auto read_string = system_metrics_collector::readFileToString(PROC_STAT_FILE_MEM);
      mem_usage = system_metrics_collector::processMemInfoLines(read_string);
    }

    sleep(1);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = finish - start;

    float seconds_running = elapsed.count() / 1000;
    printf("%5f, ", seconds_running);
    printf("%.5f, ", cpu_percentage);
    printf("%5f, ", mem_usage);
    printf("%s, ", get_capacity("/"));
    printf("%s\n", get_free_space("/"));

    if (seconds_running > timeout) {
      break;
    }

    if (m_os.is_open()) {
      m_os << seconds_running << ", " << cpu_percentage << ", " <<
        mem_usage << ", " << get_capacity("/") << ", " << get_free_space("/") <<
        std::endl;
    }
  }

  m_os.close();

  return 0;
}
