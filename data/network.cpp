#include <data/network.h>

#include <cassert>
#include <limits>

network::network(unsigned int nt, unsigned int ns, const trains& trn, const speeds& spd, const segments& seg) {
    auto large_default = std::numeric_limits<unsigned int>::max();
    
    min_time_to_arrive = uint_matrix_2d(nt, uint_vector(ns + 2, large_default));
    min_travel_time = uint_matrix_2d(nt, uint_vector(ns + 2, large_default));
    main_tracks = uint_matrix_2d(ns + 2);
    unpreferred = bool_matrix_2d(nt, bool_vector(ns + 2, false));
    connected = bool_matrix_2d(ns + 2, bool_vector(ns + 2, false));
    
    for(auto s1 = 0u; s1 <= ns + 1; s1++) {
        if(seg.type.at(s1) == 'S') {
            sidings.push_back(s1);
        }
        
        if(seg.type.at(s1) == 'X') {
            xovers.push_back(s1);
        }
        
        for(auto i = 0u; i < nt; i++) {
            if( (trn.is_westbound.at(i) && !seg.is_westbound.at(s1)) ||
                (trn.is_eastbound.at(i) && !seg.is_eastbound.at(s1))
            ) {
                unpreferred[i][s1] = true;
            }
        }
        
        for(auto s2 = 0u; s2 <= ns + 1; s2++) {
            if(seg.e_ext.at(s1) == seg.w_ext.at(s2) || seg.w_ext.at(s1) == seg.e_ext.at(s2)) {
                connected[s1][s2] = true;
            }
        }
    }
    
    calculate_times(nt, ns, trn, spd, seg);
    calculate_main_tracks(ns, seg);
    
    for(auto s1 = 0u; s1 <= ns + 1; s1++) {
        assert(connected.at(s1).at(s1) == false);
        
        for(auto s2 = s1 + 1; s2 <= ns + 1; s2++) {
            assert(connected.at(s1).at(s2) == connected.at(s2).at(s1));
        }
    }
}

auto network::calculate_times(unsigned int nt, unsigned int ns, const trains& trn, const speeds& spd, const segments& seg) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 1u; s <= ns; s++) {
            auto time_from_w = static_cast<unsigned int>(std::ceil(seg.w_min_dist.at(s) / trn.speed_max.at(i)));
            auto time_from_e = static_cast<unsigned int>(std::ceil(seg.e_min_dist.at(s) / trn.speed_max.at(i)));
            
            min_time_to_arrive[i][s] = (trn.is_westbound.at(i) ? time_from_e : time_from_w) + trn.entry_time.at(i);
            
            auto speed = 0.0;
            auto speed_aux = 0.0;
            
            if(seg.type.at(s) == '0' || seg.type.at(s) == '1' || seg.type.at(s) == '2') {
                if(trn.is_westbound.at(i)) {
                    speed = spd.ew;
                } else {
                    speed = spd.we;
                }
            } else if(seg.type.at(s) == 'S') {
                speed = spd.siding;
                speed_aux = spd.swi;
            } else if(seg.type.at(s) == 'X') {
                speed = spd.xover;
            }
            
            speed *= trn.speed_multi.at(i);
            speed_aux *= trn.speed_multi.at(i);
            
            if(seg.type.at(s) != 'S') {
                min_travel_time[i][s] = std::ceil(seg.length.at(s) / speed);
            } else {
                min_travel_time[i][s] = std::ceil(seg.original_length.at(s) / speed) +
                                        std::ceil((seg.length.at(s) - seg.original_length.at(s)) / speed_aux);
            }
        }
    }
}

auto network::calculate_main_tracks(unsigned int ns, const segments& seg) -> void {
    for(auto s = 1u; s <= ns; s++) {
        for(auto m = 1u; m <= ns; m++) {
            auto is_main = (
                seg.type.at(s) == 'S' &&
                (seg.type.at(m) == '0' || seg.type.at(m) == '1' || seg.type.at(m) == '2') &&
                (seg.e_ext.at(s) == seg.e_ext.at(m) || seg.w_ext.at(s) == seg.w_ext.at(m))
            );
            
            if(is_main) {
                main_tracks[s].push_back(m);
            }
        }
    }
}