#include "utilities/utilities.hpp"

std::vector<std::string> split(const std::string& s, char delimiter)
{
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
     if(!token.empty()) {
       std::cout << token << std::endl;
       tokens.push_back(token);
     }
   }
   return tokens;
}


std::string getPIDByName(std::string process_name, std::string my_own_name, std::string arguments)
{
  const std::string directory("/proc");

  for(auto& process: fs::directory_iterator(directory)){

    std::string line;
    std::ifstream myfile (std::string(process.path()) + std::string("/cmdline"));
    if (myfile.is_open()) {
      while ( std::getline (myfile,line) ) {
        if (line.find(process_name) != std::string::npos &&
            !(line.find(my_own_name) != std::string::npos) &&
            line.find(arguments) != std::string::npos) {
              std::cout << line << std::endl;

              std::vector<std::string> tokens_cmdline = split(line, '\0');
              std::vector<std::string> tokens_cmd = split(tokens_cmdline[0], '/');

              std::string cmd = my_own_name;
              if(tokens_cmd.size() > 0){
                std::cout << tokens_cmd[tokens_cmd.size()-1] << std::endl;
                if( !(tokens_cmd[tokens_cmd.size()-1].find(process_name) != std::string::npos)) {
                  continue;
                }
              }
            std::vector<std::string> tokens_pid = split(process.path(), '/');
            // for(auto& t: tokens)
            //   std::cout << t << std::endl;
            return tokens_pid[1];
        }
      }
    }
  }
  return std::string("");
}
