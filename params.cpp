#include <params.h>

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
        pt.get<bool>("heuristics.corridor"),
        pt.get<int>("heuristics.corridor_minutes_around_ideal"),
        pt.get<double>("heuristics.corridor_pct_around_ideal"),
        pt.get<bool>("heuristics.sparsification"),
        pt.get<int>("heuristics.sparsification_keepall_range")
    );
}