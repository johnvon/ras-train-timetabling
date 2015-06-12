#include <data/path.h>

#include <iostream>

path::path(const data& d, unsigned int train, uint_matrix_3d x, double cost) : d{&d}, train{train}, x{x}, cost{cost} {
    if(x.at(0).at(0).at(d.ns + 1) > 0u) {
        // Dummy path!
        make_dummy();
    } else {
        // Normal path!
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
        
            if(current_time >= d.ni + 1) {
                std::cerr << "Reached end of time horizon without being at the sink node!" << std::endl;
                std::cerr << "Path: ";
                for(const auto& n : p) {
                    std::cerr << "(" << n.seg << ", " << n.t << ") ";
                }
                std::cerr << std::endl;
                break;
            }
        
            current_time++;
        }
    }
}

path::path(const data& d, unsigned int train) : d{&d}, train{train}, cost{0.0} {
    make_dummy();
}

path::path(const data& d, unsigned int train, bv<node> p, double cost) : d{&d}, train{train}, p{p}, cost{cost}{
	if (p.empty()){
		make_dummy();
		std::cout << "The train " <<  d.trn.name.at(train) << " could not be scheduled" << std::endl;
	}
	else{
		x = bv<bv<bv<unsigned int>>>(d.ns+2, bv<bv<unsigned int>>(d.ni+1, bv<unsigned int> (d.ns+2,0)));
		std::cout << "Train " << d.trn.name.at(train) << ":" << std::endl;
		std::cout << "segment " << p.at(0).seg << ", at " << p.at(0).t << std::endl;
		for(unsigned int n=1; n<p.size(); n++){
			x.at(p.at(n-1).seg).at(p.at(n-1).t).at(p.at(n).seg)=1;
			std::cout << "segment " << p.at(n).seg << ", at " << p.at(n).t << std::endl;
		}
	}

}
auto path::make_dummy() -> void {
    x = uint_matrix_3d(d->ns + 2, uint_matrix_2d(d->ni + 2, uint_vector(d->ns + 2, 0u)));
    x.at(0).at(0).at(d->ns + 1) = 1u;
    p = bv<node>();
    p.push_back(node(0u, 0u));
    p.push_back(node(d->ns + 1, 1u));
    cost = 0.0;
}

auto path::make_empty() -> void {
    x = uint_matrix_3d(d->ns + 2, uint_matrix_2d(d->ni + 2, uint_vector(d->ns + 2, 0u)));
    p = bv<node>();
    cost = 0.0;
}

auto path::is_dummy() const -> bool {
    if(x.at(0).at(0).at(d->ns + 1) > 0u) {
        assert(p.size() == 2u);
    }
    
    return (x.at(0).at(0).at(d->ns + 1) > 0u);
}

auto path::is_empty() const -> bool {
    return (p.size() == 0);
}

auto path::print_summary(std::ostream& where) const -> void {
    where << "** Train: " << train << " **" << std::endl;
    
    auto current_seg = 0u;
    auto current_time = 0u;
    auto current_entry_time = -1;

    while(current_seg != d->ns + 1) {
        auto next_seg = -1;
                    
        if(current_time >= d->ni + 1) {
            std::cerr << "Could not find a valid path!" << std::endl;
            break;
        }
        
        for(auto s : d->gr.delta.at(train).at(current_seg)) {
            if(x.at(current_seg).at(current_time).at(s) > 0u) {
                next_seg = static_cast<long>(s);
                break;
            }
        }
        
        if(next_seg < 0 && current_time == d->ni) {
            if(x.at(current_seg).at(d->ni).at(d->ns + 1) > 0u) {
                next_seg = static_cast<long>(d->ns + 1);
            }
        }
            
        if(next_seg >= 0) {
            if(static_cast<unsigned int>(next_seg) != current_seg) {
                if(current_entry_time >= 0l) {
                    where << "\tLeaving at time: " << current_time << std::endl;
                    where << "\tRunning time: " << (current_time - current_entry_time + 1) << std::endl;
                    where << "\tMinimum running time: " << d->net.min_travel_time.at(train).at(current_seg) << std::endl;
                }
                
                if(next_seg != static_cast<long>(d->ns + 1)) {
                    where << "Segment " << next_seg << std::endl;
                    where << "\tEntering at time: " << current_time + 1 << std::endl;
                }
                
                current_entry_time = static_cast<long>(current_time + 1);
            }
            current_seg = next_seg;
        }
        
        current_time++;
    }
}
