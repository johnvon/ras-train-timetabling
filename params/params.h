#ifndef PARAMS_H
#define PARAMS_H

#include <string>

struct params {
    struct cplex_params {
        int threads;
        int time_limit;
        
        cplex_params() {}
        cplex_params(int threads, int time_limit) : threads{threads}, time_limit{time_limit} {}
    };
    
    struct model_params {
        bool alternative_min_travel_time_cst;
        bool max_travel_time_cst;
    
        model_params() {}
        model_params(bool amin_cst, bool max_cst) : alternative_min_travel_time_cst{amin_cst}, max_travel_time_cst{max_cst} {}
    };

    struct heuristics_params {
        struct corridor_params {
            bool active;
            int minutes_around_ideal;
            double pct_around_ideal;
            
            corridor_params() {}
            corridor_params(bool active, int minutes_around_ideal, double pct_around_ideal) : active{active}, minutes_around_ideal{minutes_around_ideal}, pct_around_ideal{pct_around_ideal} {}
        };
        
        struct sparsification_params {
            bool active;
            int keepall_range;
            
            sparsification_params() {}
            sparsification_params(bool active, int keepall_range) : active{active}, keepall_range{keepall_range} {}
        };
        
        struct constructive_params {
            bool active;
            bool fix_start;
            bool fix_end;
            bool only_start_at_main;
            
            constructive_params() {}
            constructive_params(bool active, bool fix_start, bool fix_end, bool only_start_at_main) : active{active}, fix_start{fix_start}, fix_end{fix_end}, only_start_at_main{only_start_at_main} {}
        };
        
        bool                    simplified_objective_function;
        corridor_params         corridor;
        sparsification_params   sparsification;
        constructive_params     constructive;
    
        heuristics_params() {}
        heuristics_params(bool simplified_objective_function, corridor_params corridor, sparsification_params sparsification, constructive_params constructive) : simplified_objective_function{simplified_objective_function}, corridor{std::move(corridor)}, sparsification{std::move(sparsification)}, constructive{std::move(constructive)} {}
    };
    
    std::string         results_file;
    cplex_params        cplex;
    model_params        model;
    heuristics_params   heuristics;
    
    params(std::string file_name);
};

#endif