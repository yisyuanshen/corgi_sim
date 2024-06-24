#include "tools.hpp"

vector<vector<double>> read_csv(string input_filename){
    vector<vector<double>> data;
    ifstream input_file(input_filename);
    string line;

    if (!input_file.is_open()) {
        cerr << "Could not open the input file: " << input_filename << endl;
        return data;
    }

    while (getline(input_file, line)) {
        istringstream iss(line);
        vector<double> row;
        string value;
    
        while (getline(iss, value, ',')) {
            try { row.push_back(stod(value)); } 
            catch (const invalid_argument& ia) { cerr << "Invalid argument: " << ia.what() << '\n'; }
        }
        
        if (row.size() == 8) { data.push_back(row); } 
        else { cerr << "Row with incorrect number of elements encountered.\n"; }
    }

    input_file.close();

    return data;
}


void write_csv(const std::string& output_filename, const std::vector<std::vector<std::string>>& data) {
    std::ofstream output_file(output_filename);
    
    if (!output_file.is_open()) {
        std::cerr << "Could not open the output file: " << output_filename << std::endl;
        return;
    }

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            output_file << row[i];
            if (i != row.size() - 1) output_file << ",";
        }
        output_file << "\n";
    }

    output_file.close();
    std::cout << "Data successfully written to " << output_filename << std::endl;
}
