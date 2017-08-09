#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
#include <stdlib.h>

int main (int argc, char** argv)
{
  std::ifstream infile("obstacles3.txt");
  std::ofstream myfile;
  myfile.open("obstacles2.txt");
  for(std::string line; getline(infile, line);) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss),
      std::istream_iterator<std::string>(),
      std::back_inserter(tokens));
    if (tokens.size() == 1)
      myfile << line << "\n";
    else if (tokens.size() == 2) {
      double x = atof(tokens.at(0).c_str()) + 1.40;
      double y = -1 * atof(tokens.at(1).c_str()) + 7.326;
      myfile << x << " " << y << "\n";
    }
  }
  myfile.close();
  return (0);
}