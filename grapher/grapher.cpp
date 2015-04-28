#include <grapher/grapher.h>
#include <grapher/gnuplot-iostream.h>

auto grapher::generate_points() -> grapher::points_data {
    auto points = points_data(d.nt);
    
    for(auto i = 0u; i < d.nt; i++) {
        auto current_seg = 0u;
        auto current_time = 0u;
        auto current_entry_time = -1;
        points.at(i) = series_data();
    
        while(current_seg != d.ns + 1) {
            auto next_seg = -1;
            auto escaping = false;
            
            for(auto s : d.gr.delta[i][current_seg]) {
                if(x.at(i).at(current_seg).at(current_time).at(s) > 0u) {
                    next_seg = static_cast<long>(s);
                }
            }
            
            if(next_seg < 0 && current_time == d.ni) {
                if(x.at(i).at(current_seg).at(d.ni).at(d.ns + 1) > 0u) {
                    next_seg = static_cast<long>(d.ns + 1);
                    escaping = true;
                }
            }
            
            if(current_time >= d.ni + 1) {
                std::cerr << "Could not find a valid path!" << std::endl;
                break;
            }
                
            if(next_seg >= 0) {
                if(static_cast<unsigned int>(next_seg) != current_seg) {
                    if(current_entry_time >= 0l) {
                        // Coming from another segment
                        
                        if(next_seg == static_cast<long>(d.ns + 1)) {
                            // If arriving in tau
                            
                            if(escaping) {
                                // If escaping
                                
                                auto distance = (
                                    d.trn.is_eastbound.at(i) ?
                                    d.seg.w_min_dist.at(current_seg) + d.seg.length.at(current_seg) :
                                    d.seg.w_min_dist.at(current_seg)
                                );
                                points.at(i).push_back(std::make_pair(current_time, distance));
                            } else {
                                // If concluding the journey
                                
                                auto distance = (
                                    d.trn.is_eastbound.at(i) ?
                                    d.seg.w_min_dist.at(current_seg) + d.seg.length.at(current_seg) : 
                                    0
                                );
                                points.at(i).push_back(std::make_pair(current_time, distance));
                            }
                        } else {
                            // If arriving at a normal segment
                            
                            auto distance = (
                                d.trn.is_eastbound.at(i) ?
                                d.seg.w_min_dist.at(next_seg) :
                                d.seg.w_min_dist.at(current_seg)
                            );
                            points.at(i).push_back(std::make_pair(current_time, distance));
                        }
                    } else {
                        // If just starting its journey
                        
                        auto distance = (
                            d.trn.is_eastbound.at(i) ?
                            0 :
                            d.seg.w_min_dist.at(next_seg) + d.seg.length.at(next_seg)
                        );
                        points.at(i).push_back(std::make_pair(current_time, distance));
                    }
                    
                    current_entry_time = static_cast<long>(current_time + 1);
                }
                current_seg = next_seg;
            }
            
            current_time++;
        }
    }
    
    return points;
}

auto grapher::write_graph() -> void {
    auto points = generate_points();
    Gnuplot gp;
    
    gp << "set terminal png size 2048" << std::endl;
    gp << "set output \"graph.png\"" << std::endl;
    
    for(auto i = 0u; i < d.nt; i++) {
        gp << "plot '-' title \"Train " << i << "\" with lines" << std::endl;
        gp.send1d(points.at(i));
        if(i != d.nt - 1) {
            gp << ", ";
        }
    }
}