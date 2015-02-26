#include <params/params.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

params::params(std::string file_name) {
    using namespace boost::property_tree;
    
    auto pt = ptree();
    read_json(file_name, pt);
    
    results_file = pt.get<std::string>("results_file");
    
    cplex = cplex_params(
        pt.get<unsigned int>("cplex.threads"),
        pt.get<unsigned int>("cplex.time_limit")
    );
        
    heuristics = heuristics_params(
        heuristics_params::mip_constructive_params(
            pt.get<bool>("heuristics.constructive.active"),
            pt.get<bool>("heuristics.constructive.fix_start"),
            pt.get<bool>("heuristics.constructive.fix_end"),
            pt.get<bool>("heuristics.constructive.only_start_at_main")
        )
    );
}