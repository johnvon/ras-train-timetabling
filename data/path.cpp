#include <data/path.h>

#include <iostream>

path::path(const data& d, unsigned int train, uint_matrix_3d x) : d{d}, train{train}, x{x} {
    auto current_seg = 0u;
    auto current_time = 0u;
    
    while(current_seg != d.ns + 1) {
        auto next_seg = -1;
        
        for(auto s : d.gr.delta.at(train).at(current_seg)) {
            if(x.at(current_seg).at(current_time).at(s) > 0u) {
                next_seg = static_cast<long>(s);
                break;
            }
        }
        
        if(next_seg < 0 && current_time == d.ni) {
            if(x.at(current_seg).at(d.ni).at(d.ns + 1) > 0u) {
                next_seg = static_cast<long>(d.ns + 1);
            }
        }
        
        if(next_seg >= 0) {
            if(p.size() == 0) {
                p.push_back({0, current_time});
            }
            p.push_back({static_cast<unsigned int>(next_seg), current_time + 1});
            current_seg = next_seg;
        }
        
        assert(current_time <= d.ni + 1);
        
        current_time++;
    }
}

auto path::print_summary() const -> void {
    std::cout << "** Train: " << train << " **" << std::endl;
    
    auto current_seg = 0u;
    auto current_time = 0u;
    auto current_entry_time = -1;

    while(current_seg != d.ns + 1) {
        auto next_seg = -1;
                    
        if(current_time >= d.ni + 1) {
            std::cerr << "Could not find a valid path!" << std::endl;
            break;
        }
        
        for(auto s : d.gr.delta.at(train).at(current_seg)) {
            if(x.at(current_seg).at(current_time).at(s) > 0u) {
                next_seg = static_cast<long>(s);
                break;
            }
        }
        
        if(next_seg < 0 && current_time == d.ni) {
            if(x.at(current_seg).at(d.ni).at(d.ns + 1) > 0u) {
                next_seg = static_cast<long>(d.ns + 1);
            }
        }
            
        if(next_seg >= 0) {
            if(static_cast<unsigned int>(next_seg) != current_seg) {
                if(current_entry_time >= 0l) {
                    std::cout << "\tLeaving at time: " << current_time << std::endl;
                    std::cout << "\tRunning time: " << (current_time - current_entry_time + 1) << std::endl;
                    std::cout << "\tMinimum running time: " << d.net.min_travel_time.at(train).at(current_seg) << std::endl;
                }
                
                if(next_seg != static_cast<long>(d.ns + 1)) {
                    std::cout << "Segment " << next_seg << std::endl;
                    std::cout << "\tEntering at time: " << current_time + 1 << std::endl;
                }
                
                current_entry_time = static_cast<long>(current_time + 1);
            }
            current_seg = next_seg;
        }
        
        current_time++;
    }
}