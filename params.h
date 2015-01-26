#ifndef PARAMS_H
#define PARAMS_H

#include <string>

struct params {
    struct model_params {
        bool alternative_min_travel_time_cst;
        bool max_travel_time_cst;
    
        model_params() {}
        model_params(bool amin_cst, bool max_cst) : alternative_min_travel_time_cst{amin_cst}, max_travel_time_cst{max_cst} {}
    };

    struct heuristics_params {
        bool simplified_objective_function;
    
        heuristics_params() {}
        heuristics_params(bool simp_obj) : simplified_objective_function{simp_obj} {}
    };
    
    model_params model;
    heuristics_params heuristics;
    
    params(std::string file_name);
};

#endif