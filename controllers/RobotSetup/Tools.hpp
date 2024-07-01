#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <mutex>

using namespace std;

vector<vector<double>> read_csv(string input_filename);
void write_csv(const std::string& output_filename, const std::vector<std::vector<std::string>>& data);

#endif