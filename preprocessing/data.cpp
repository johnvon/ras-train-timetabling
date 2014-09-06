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
            schedule.emplace(node, ti);
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
    std::cout << "Instance: ``" << instance_name << "''" << std::endl;
    std::cout << "Segments: " << segments_number << ", trains: " << trains_number << ", time intervals: " << time_intervals << std::endl;
    std::cout << "Speeds. Ew: " << speed_ew << ", we: " << speed_we << ", siding: " << speed_siding << ", switch: " << speed_switch << ", xover: " << speed_xover << std::endl;
    std::cout << "Prices. Terminal: " << terminal_delay_price << ", schedule: " << schedule_delay_price << ", Unpreferred: " << unpreferred_price << std::endl;
    std::cout << "General delay price. ";
    for(char cl = 'A'; cl <= 'F'; cl++) { std::cout << cl << ": " << general_delay_price.at(cl) << ", "; }
    std::cout << std::endl;
}

bool Data::is_main(std::shared_ptr<const Segment> m, std::shared_ptr<const Segment> s) const {
    if(s->type != 'S' || (m->type != '0' && m->type != '1' && m->type != '2')) {
        return false;
    }
    
    std::vector<std::shared_ptr<const Segment>> switches;
    
    for(auto ss : segments) {
        if(ss->type == 'T' && (ss->extreme_1 == s->extreme_2 || ss->extreme_2 == s->extreme_1)) {
            switches.push_back(ss);
        }
    }
    
    if(switches.size() != 2) { throw std::runtime_error("Found a siding that doesn't have two switches!"); }
    
    for(auto sw : switches) {
        if(sw->extreme_1 == m->extreme_1 || sw->extreme_2 == m->extreme_2) {
            return true;
        }
    }
    
    return false;
}

int Data::segment_id_for_node(int node_number, bool westbound_train) const {
    for(auto segment : segments) {
        if((westbound_train && segment->extreme_1 == node_number) || (!westbound_train && segment->extreme_2 == node_number)) {
            return segment->id;
        }
    }
    throw std::runtime_error("Couldn't find a segment for a SA node to be visited through!");
}