#include <data/trains.h>

#include <boost/foreach.hpp>

#include <cassert>

using namespace boost::property_tree;
using namespace boost;

constexpr unsigned int trains::heavy_weight;
constexpr char trains::first_train_class;
constexpr char trains::last_train_class;

trains::trains(const ptree& pt, unsigned int nt, unsigned int ns, unsigned int ni, const speeds& spd, const segments& seg) {
    BOOST_FOREACH(const ptree::value_type& train_child, pt.get_child("trains")) {
        want_time.push_back(train_child.second.get<unsigned int>("terminal_wt"));
        entry_time.push_back(train_child.second.get<unsigned int>("entry_time"));
        tob.push_back(train_child.second.get<unsigned int>("tob"));
        speed_multi.push_back(train_child.second.get<double>("speed_multi"));
        
        // speed_max empty for now
        speed_max.push_back(0.0);
        
        length.push_back(train_child.second.get<double>("length"));
        type.push_back(train_child.second.get<char>("class"));
        is_sa.push_back(train_child.second.get<bool>("schedule_adherence"));
        is_heavy.push_back(tob.back() > trains::heavy_weight);
        is_eastbound.push_back(train_child.second.get<bool>("eastbound"));
        is_westbound.push_back(train_child.second.get<bool>("westbound"));
        is_hazmat.push_back(train_child.second.get<bool>("hazmat"));
        orig_ext.push_back(train_child.second.get<unsigned int>("origin_node"));
        dest_ext.push_back(train_child.second.get<unsigned int>("destination_node"));
        
        // orig_segs, dest_segs, unpreferred_segs empty for now
        orig_segs.push_back(uint_vector());
        dest_segs.push_back(uint_vector());
        unpreferred_segs.push_back(uint_vector());
        
        sa_ext.push_back(uint_vector());
        sa_times.push_back(uint_vector());
        BOOST_FOREACH(const ptree::value_type& schedule_child, train_child.second.get_child("schedule")) {
            auto extreme = schedule_child.second.get<unsigned int>("node");
            auto ti = schedule_child.second.get<unsigned int>("time");
            
            if(ti <= ni) {
                sa_ext.back().push_back(extreme);
                sa_times.back().push_back(ti);
            }
        }
        
        sa_num.push_back(sa_ext.back().size());
        
        // sa_segs empty for now
        sa_segs.push_back(uint_matrix_2d(sa_num.back(), uint_vector()));
        
        assert(want_time.back() < ni);
        assert(entry_time.back() < ni);
        assert(want_time.back() >= entry_time.back());
        assert(speed_multi.back() > 0);
        assert(length.back() > 0);
        assert(type.back() >= first_train_class && type.back() <= last_train_class);
        assert(is_sa.back() != (sa_num.back() == 0));
        assert(is_eastbound.back() != is_westbound.back());
    }
    
    calculate_max_speeds(nt, spd);
    calculate_origin_and_destination_segments(nt, ns, seg);
    calculate_unpreferred_segments(nt, ns, seg);
    calculate_sa_segments(nt, ns, seg);
}

auto trains::calculate_max_speeds(unsigned int nt, const speeds& spd) -> void {
    for(auto i = 0u; i < nt; i++) {
        speed_max.at(i) = speed_multi.at(i) * (is_westbound.at(i) ? spd.ew : spd.we);
    }
}

auto trains::calculate_origin_and_destination_segments(unsigned int nt, unsigned int ns, const segments& seg) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 0u; s <= ns + 1; s++) {            
            if(is_eastbound.at(i) && seg.w_ext.at(s) == orig_ext.at(i)) {
                orig_segs.at(i).push_back(s);
            }
            if(is_westbound.at(i) && seg.e_ext.at(s) == orig_ext.at(i)) {
                orig_segs.at(i).push_back(s);
            }
            if(is_eastbound.at(i) && seg.e_ext.at(s) == dest_ext.at(i)) {
                dest_segs.at(i).push_back(s);
            }
            if(is_westbound.at(i) && seg.w_ext.at(s) == dest_ext.at(i)) {
                dest_segs.at(i).push_back(s);
            }
        }
    }
}

auto trains::calculate_unpreferred_segments(unsigned int nt, unsigned int ns, const segments& seg) -> void {
    for(auto i = 0u; i < nt; i++) {
        for(auto s = 0u; s <= ns + 1; s++) {
            if( (is_westbound.at(i) && !seg.is_westbound.at(s)) ||
                (is_eastbound.at(i) && !seg.is_eastbound.at(s))
            ) {
                unpreferred_segs.at(i).push_back(s);
            }
        }
    }
}

auto trains::calculate_sa_segments(unsigned int nt, unsigned int ns, const segments& seg) -> void {
    for(auto i = 0u; i < nt; i++) {
        if(is_sa.at(i)) {
            for(auto n = 0u; n < sa_num.at(i); n++) {
                auto at_least_one_segment_found = false;
                
                for(auto s = 0u; s <= ns + 1; s++) {
                    if( (is_westbound.at(i) && seg.w_ext.at(s) == sa_ext.at(i).at(n)) ||
                        (is_eastbound.at(i) && seg.e_ext.at(s) == sa_ext.at(i).at(n))
                    ) {
                        sa_segs.at(i).at(n).push_back(s);
                        at_least_one_segment_found = true;
                    }
                }
                
                assert(at_least_one_segment_found);
            }
        }
    }
}