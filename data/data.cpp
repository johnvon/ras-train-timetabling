#include <data/data.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <utility>

#include <boost/property_tree/json_parser.hpp>

using namespace boost::property_tree;
using namespace boost;

data::data(const std::string& file_name, const params& p) : p{p} {
    using namespace std::chrono;
    
    ptree pt;
    read_json(file_name, pt);
    
    ins = instance(pt.get<std::string>("name"), file_name);
    
    nt = pt.get<unsigned int>("trains_number");
    ns = pt.get<unsigned int>("segments_number");
    ni = pt.get<unsigned int>("time_intervals");
    
    spd = speeds(pt);
    seg = segments(pt);
    mnt = mows(pt, ni, ns, seg);
    tiw = time_windows(pt, ni);
    pri = prices(pt);
    trn = trains(pt, nt, ns, ni, spd, seg);
        
    auto t_start = high_resolution_clock::now();

    net = network(nt, ns, trn, spd, seg);
    gr = graph(nt, ns, ni, p, trn, mnt, seg, net, tiw, pri);

    auto t_end = high_resolution_clock::now();
    auto time_span = duration_cast<duration<double>>(t_end - t_start);
    
    std::cout << "Graphs creation: " << time_span.count() << " seconds" << std::endl;
    
    for(auto i = 0u; i < nt; i++) {
        std::cout << "Graph for train " << i << std::endl;
        std::cout << "\t" << gr.n_nodes.at(i) << " nodes" << std::endl;
        std::cout << "\t" << gr.n_arcs.at(i) << " arcs" << std::endl;
    }
}