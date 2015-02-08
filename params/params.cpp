#include <params/params.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

params::params(std::string file_name) {
    using namespace boost::property_tree;
    
    auto pt = ptree();

    read_json(file_name, pt);
    
    results_file = pt.get<std::string>("results_file");
    
    cplex = cplex_params(
        pt.get<int>("cplex.threads"),
        pt.get<int>("cplex.time_limit")
    );
    
    model = model_params(
        pt.get<bool>("model.alternative_min_travel_time_cst"),
        pt.get<bool>("model.max_travel_time_cst")
    );
        
    heuristics = heuristics_params(
        pt.get<bool>("heuristics.simplified_objective_function"),
        heuristics_params::corridor_params(
            pt.get<bool>("heuristics.corridor.active"),
            pt.get<int>("heuristics.corridor.minutes_around_ideal"),
            pt.get<double>("heuristics.corridor.pct_around_ideal")
        ),
        heuristics_params::sparsification_params(
            pt.get<bool>("heuristics.sparsification.active"),
            pt.get<int>("heuristics.sparsification.keepall_range")         
        ),
        heuristics_params::constructive_params(
            pt.get<bool>("heuristics.constructive.active"),
            pt.get<bool>("heuristics.constructive.fix_start"),
            pt.get<bool>("heuristics.constructive.fix_end"),
            pt.get<bool>("heuristics.constructive.only_start_at_main")
        )
    );
}