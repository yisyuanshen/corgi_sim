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

