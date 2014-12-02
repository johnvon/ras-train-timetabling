#include <params.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

params::params(std::string file_name) {
    using namespace boost::property_tree;
    
    auto pt = ptree();

    read_json(file_name, pt);
    
    model = model_params(
        pt.get<bool>("model.alternative_min_travel_time_cst"),
        pt.get<bool>("model.max_travel_time_cst")
    );
        
    heuristics = heuristics_params(
        pt.get<bool>("heuristics.simplified_objective_function")
    );
}