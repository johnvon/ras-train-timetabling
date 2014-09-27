#include <preprocessing/data.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include <iostream>
#include <stdexcept>

Data::Data(const std::string file_name) : file_name{file_name} {
    parse();
}

void Data::parse() {
    using namespace boost::property_tree;
    using namespace boost;
    
    ptree pt;
    read_json(file_name, pt);
    
    instance_name = pt.get<std::string>("name");
    
    speed_ew = pt.get<double>("speed_ew");
    speed_we = pt.get<double>("speed_we");
    speed_siding = pt.get<double>("speed_siding");
    speed_switch = pt.get<double>("speed_switch");
    speed_xover = pt.get<double>("speed_xover");
    
    time_intervals = pt.get<int>("time_intervals");
    
    for(char cl = 'A'; cl <= 'F'; cl++) {
        double price {pt.get_child("general_delay_price").get<double>(std::string(1,cl))};
        general_delay_price.emplace(cl, price);
    }
    
    terminal_delay_price = pt.get<double>("terminal_delay_price");
    schedule_delay_price = pt.get<double>("schedule_delay_price");
    unpreferred_price = pt.get<double>("unpreferred_price");
    
    segments_number = pt.get<int>("segments_number");
    int segment_id {0};
    
    BOOST_FOREACH(const ptree::value_type& segment_child, pt.get_child("segments")) {
        int extreme_1 {segment_child.second.get<int>("extreme_1")};
        int extreme_2 {segment_child.second.get<int>("extreme_2")};
        double min_distance_from_w {segment_child.second.get<double>("min_distance_from_w")};
        double min_distance_from_e {segment_child.second.get<double>("min_distance_from_e")};
        char type {segment_child.second.get<char>("type")};
        double length {segment_child.second.get<double>("length")};
        bool eastbound {segment_child.second.get<bool>("eastbound")};
        bool westbound {segment_child.second.get<bool>("westbound")};
        
        Segment s {segment_id++, extreme_1, extreme_2, min_distance_from_w, min_distance_from_e, type, length, eastbound, westbound};
        segments.push_back(std::make_shared<const Segment>(s));
    }
    
    trains_number = pt.get<int>("trains_number");
    want_time_tw_start = pt.get<int>("want_time_tw_start");
    want_time_tw_end = pt.get<int>("want_time_tw_end");
    schedule_tw_end = pt.get<int>("schedule_tw_end");
    headway = pt.get<int>("headway");
    int train_id {0};
    
    BOOST_FOREACH(const ptree::value_type& train_child, pt.get_child("trains")) {
        char cl {train_child.second.get<char>("class")};
        bool schedule_adherence {train_child.second.get<bool>("schedule_adherence")};
        int entry_time {train_child.second.get<int>("entry_time")};
        int origin_node {train_child.second.get<int>("origin_node")};
        int destination_node {train_child.second.get<int>("destination_node")};
        bool eastbound {train_child.second.get<bool>("eastbound")};
        bool westbound {train_child.second.get<bool>("westbound")};
        double speed_multi {train_child.second.get<double>("speed_multi")};
        double length {train_child.second.get<double>("length")};
        int tob {train_child.second.get<int>("tob")};
        bool hazmat {train_child.second.get<bool>("hazmat")};
        int terminal_wt {train_child.second.get<int>("terminal_wt")};
        Schedule schedule;
        BOOST_FOREACH(const ptree::value_type& schedule_child, train_child.second.get_child("schedule")) {
            int node {schedule_child.second.get<int>("node")};
            int ti {schedule_child.second.get<int>("time")};
            schedule.emplace(segment_id_for_node(node, westbound), ti);
        }
        
        Train t {train_id++, cl, schedule_adherence, entry_time, origin_node, destination_node, eastbound, westbound, speed_multi, length, tob, hazmat, terminal_wt, schedule};
        trains.push_back(t);
    }
    
    BOOST_FOREACH(const ptree::value_type& mow_child, pt.get_child("mow")) {
        int extreme_1 {mow_child.second.get<int>("extreme_1")};
        int extreme_2 {mow_child.second.get<int>("extreme_2")};
        int start_time {mow_child.second.get<int>("start_time")};
        int end_time {mow_child.second.get<int>("end_time")};
        
        Mow m {extreme_1, extreme_2, start_time, end_time};
        mow.push_back(m);
    }
}

void Data::print_stats() const {
    std::cout << "Instance: ``" << instance_name << "''" << std::endl << std::endl;
    
    std::cout << "****** GENERAL DATA ******" << std::endl << std::endl;
    std::cout << "Segments: " << segments_number << ", trains: " << trains_number << ", time intervals: " << time_intervals << std::endl;
    std::cout << "Speeds. Ew: " << speed_ew << ", we: " << speed_we << ", siding: " << speed_siding << ", switch: " << speed_switch << ", xover: " << speed_xover << std::endl << std::endl;
    
    std::cout << "****** COSTS ******" << std::endl << std::endl;
    std::cout << "Prices. Terminal: " << terminal_delay_price << ", schedule: " << schedule_delay_price << ", unpreferred: " << unpreferred_price << std::endl;
    std::cout << "General delay price. ";
    for(char cl = 'A'; cl <= 'F'; cl++) { std::cout << cl << ": " << general_delay_price.at(cl) << ", "; }
    std::cout << std::endl << std::endl;
    
    std::cout << "****** TOPOLOGY ******" << std::endl << std::endl;
    std::cout << "Segments: " << std::endl;
    for(auto s : segments) {
        std::cout << "\tSegment #" << s->id << " (" << s->type << "); length: " << s->length << "; extremes: " << s->extreme_1 << " and " << s->extreme_2 << "; min dist: " << s->min_distance_from_w << " from W and " << s->min_distance_from_e << " from E" << std::endl;
        if(s->type == 'S') {
            for(auto m : segments) {
                if(is_main(m, s)) {
                    std::cout << "\t\tThe corresponding main segment is: " << m->id << " from " << m->extreme_1 << " to " << m->extreme_2 << std::endl;
                }
            }
        }
    }
    
    // The following only works if there is no "hole" in the node numbering!
    // std::cout << "Segments from nodes: " << std::endl;
    // int node_number {0};
    // bool done_w {false};
    // bool done_e {false};
    //
    // while(!done_w || !done_e) {
    //     try {
    //         done_w = false;
    //         int segw = segment_id_for_node(node_number, true);
    //         std::cout << "\tNode " << node_number << "; segment arriving from e: " << segw << std::endl;
    //     } catch(...) {
    //         done_w = true;
    //     }
    //
    //     try {
    //         done_e = false;
    //         int sege = segment_id_for_node(node_number, false);
    //         std::cout << "\tNode " << node_number << "; segment arriving from w: " << sege << std::endl;
    //     } catch(...) {
    //         done_e = true;
    //     }
    //
    //     node_number++;
    // }
}

bool Data::is_main(std::shared_ptr<const Segment> m, std::shared_ptr<const Segment> s) const {
    if(s->type != 'S' || (m->type != '0' && m->type != '1' && m->type != '2')) {
        return false;
    }
    
    std::shared_ptr<const Segment> switch_e = nullptr, switch_w = nullptr;
    
    for(auto ss : segments) {
        if(ss->type == 'T') {
            if(ss->extreme_1 == s->extreme_2) {
                switch_e = ss;
            }
            if(ss->extreme_2 == s->extreme_1) {
                switch_w = ss;
            }
        }
    }
    
    if(switch_e == nullptr || switch_w == nullptr) { throw std::runtime_error("Found a siding that doesn't have two switches!"); }
    
    std::shared_ptr<const Segment> next_to_switch_e = nullptr, next_to_switch_w = nullptr;
    
    for(auto ss : segments) {
        if(ss->type == '0' || ss->type == '1' || ss->type == '2') {
            if(ss->extreme_1 == switch_w->extreme_1) {
                if(ss->id == m->id) {
                    return true;
                } else {
                    next_to_switch_w = ss;
                }
            } 
            if(ss->extreme_2 == switch_e->extreme_2) {
                if(ss->id == m->id) {
                    return true;
                } else {
                    next_to_switch_e = ss;
                }
            }
        }
    }
    
    if(next_to_switch_e == nullptr || next_to_switch_w == nullptr) { throw std::runtime_error("Found less than 2 next_to_switches!"); }
    
    return (m->extreme_1 == next_to_switch_w->extreme_2 || m->extreme_2 == next_to_switch_e->extreme_1);
}

int Data::segment_id_for_node(int node_number, bool westbound_train) const {
    for(auto segment : segments) {
        if((westbound_train && segment->extreme_1 == node_number) || (!westbound_train && segment->extreme_2 == node_number)) {
            return segment->id;
        }
    }
    throw std::runtime_error("Couldn't find a segment for a SA node to be visited through!");
}