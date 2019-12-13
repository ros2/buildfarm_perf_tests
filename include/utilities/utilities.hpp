#ifndef LINUX_MEASUREMENTS__UTILITIES_HPP_
#define LINUX_MEASUREMENTS__UTILITIES_HPP_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#if __cplusplus == 201402L
#include <experimental/filesystem> // C++14
namespace fs = std::experimental::filesystem; // C++14
#elif __cplusplus > 201402L
#include <filesystem> // C++17
namespace fs = std::filesystem;
#endif

std::string getPIDByName(std::string process_name, std::string my_own_name, std::string arguments);
std::vector<std::string> split(const std::string& s, char delimiter);

#endif
