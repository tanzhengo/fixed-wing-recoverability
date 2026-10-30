#define LearnedRecoveryNetwork FrozenLearnedRecoveryNetwork
#include "learned_recovery.h"
#undef LearnedRecoveryNetwork
#include "learned_recovery_revision.h"
#include <sstream>
#include <iostream>
#include <iomanip>
int main(int argc,char** argv) {
    if(argc!=4)return 2;
    FrozenLearnedRecoveryNetwork old_model;LearnedRecoveryNetwork compact;
    if(!old_model.load(argv[1])||!compact.load(argv[2]))return 3;
    std::ifstream f(argv[3]);std::string row;std::getline(f,row);int feature_count=0;{
        std::istringstream names(row);std::string name;while(std::getline(names,name,','))if(name.size()>1&&name[0]=='f'&&std::isdigit(name[1]))++feature_count;
    }
    if(feature_count!=28&&feature_count!=103)return 4;
    int count=0;double max_error=0,max_reference=0;
    while(std::getline(f,row)) {
        std::replace(row.begin(),row.end(),',',' ');std::istringstream in(row);
        std::vector<float> x(103,0.0f);float expected;
        for(int i=0;i<feature_count;++i)if(!(in>>x[i]))return 4;
        if(!(in>>expected))return 4;
        std::array<float,2> before,after;
        if(!old_model.predict(x,before))return 5;
        x.resize(compact.feature_size());if(!compact.predict(x,after))return 5;
        max_error=std::max(max_error,double(std::abs(before[0]-after[0])));
        max_reference=std::max(max_reference,double(std::abs(expected-after[0])));++count;
    }
    std::cout<<std::setprecision(12)<<"{\"examples\":"<<count<<",\"compact_vs_padded_max_abs_error\":"<<max_error<<",\"compact_vs_python_max_abs_error\":"<<max_reference<<"}\n";
    return count>0&&max_error<1e-5&&max_reference<5e-5?0:6;
}
