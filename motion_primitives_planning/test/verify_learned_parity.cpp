// Standalone PyTorch/export parity check: g++ -O2 -std=c++11 this_file.cpp -o verify_parity
#include "../src/learned_recovery.h"
#include <cstdlib>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <sstream>

int main(int argc, char** argv) {
    if (argc != 3) { std::cerr << "usage: verify_parity MODEL PARITY_CSV\n"; return 2; }
    LearnedRecoveryNetwork network;
    if (!network.load(argv[1])) { std::cerr << "invalid model\n"; return 2; }
    std::ifstream csv(argv[2]); std::string line;
    if (!std::getline(csv,line)) { std::cerr << "empty parity file\n"; return 2; }
    size_t rows = 0; std::array<float,2> maximum = {{0,0}};
    while (std::getline(csv,line)) {
        if(line.empty()) continue;
        std::istringstream row(line); std::string token; std::vector<float> values;
        while(std::getline(row,token,',')) {
            char* end = nullptr; float value = std::strtof(token.c_str(), &end);
            while (end && std::isspace(static_cast<unsigned char>(*end))) ++end;
            if(end == token.c_str() || *end != '\0' || !std::isfinite(value)) {
                std::cerr << "invalid numeric CSV field\n"; return 2;
            }
            values.push_back(value);
        }
        if(values.size() != 105) {std::cerr<<"expected 103 features + 2 raw outputs, got "<<values.size()<<'\n';return 2;}
        std::vector<float> x(values.begin(),values.begin()+103); std::array<float,2> y;
        if(!network.predict(x,y)) return 2;
        for(size_t j=0;j<2;++j) maximum[j]=std::max(maximum[j],std::abs(y[j]-values[103+j]));
        ++rows;
    }
    std::cout << std::setprecision(10) << "{\"rows\":" << rows
              << ",\"max_logit_error\":" << maximum[0]
              << ",\"max_log_steps_error\":" << maximum[1]
              << ",\"tolerance\":0.0001,\"passed\":"
              << ((rows > 0 && maximum[0] <= 1e-4 && maximum[1] <= 1e-4) ? "true" : "false") << "}\n";
    return rows > 0 && maximum[0] <= 1e-4 && maximum[1] <= 1e-4 ? 0 : 1;
}
