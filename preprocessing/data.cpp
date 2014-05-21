#ifndef DATA_CPP
#define DATA_CPP

#include <memory>
using std::make_shared;
#include <utility>
using std::make_pair;
#include <iostream>
using std::cout;
using std::endl;

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
using namespace boost::property_tree;
#include <boost/foreach.hpp>
using namespace boost;

#include <preprocessing/data.h>

Data::Data(const string data_file_name) {
    ptree pt;
    read_json(data_file_name, pt);
    
    num_trains = pt.get<int>("trains_number");
    num_junctions = pt.get<int>("nodes_number");
    if(DEV) {
        num_times = 150; // Smaller graph to speed up execution during development
    } else {
        num_times = 60 * 12; // 1 day, 1 time interval per minute
    }
    
    for(int i = 0; i < num_junctions; ++i) {
        bool junction_added = false;
        BOOST_FOREACH(const ptree::value_type& w_term_child, pt.get_child("terminal_w")) {
            if(w_term_child.second.get<int>("") == i) {
                junctions.push_back(make_shared<Junction>(i, true, TerminalType::WEST));
                junction_added = true;
            }
        }
        if(junction_added) continue;
        BOOST_FOREACH(const ptree::value_type& e_term_child, pt.get_child("terminal_e")) {
            if(e_term_child.second.get<int>("") == i) {
                junctions.push_back(make_shared<Junction>(i, true, TerminalType::EAST));
                junction_added = true;
            }
        }
        if(junction_added) continue;
        junctions.push_back(make_shared<Junction>(i, false, TerminalType::NONE));
    }
    
    speed_ew = pt.get<float>("speed_ew");
    speed_we = pt.get<float>("speed_we");
    speed_siding = pt.get<float>("speed_siding");
    speed_switch = pt.get<float>("speed_switch");
    speed_xover = pt.get<float>("speed_xover");
    
    BOOST_FOREACH(const ptree::value_type& seg_child, pt.get_child("segments")) {
        std::shared_ptr<Junction> extreme_w = junction_by_id(seg_child.second.get<int>("extreme_w"));
        std::shared_ptr<Junction> extreme_e = junction_by_id(seg_child.second.get<int>("extreme_e"));
        char type_chr = seg_child.second.get<char>("type");
        float length = seg_child.second.get<float>("length");
        vector<float> speeds = {speed_we, speed_ew, speed_siding, speed_switch, speed_xover};
        
        tracks.push_back(make_shared<Track>(length, speeds, type_chr, TrackDirection::EASTBOUND, make_pair(extreme_w, extreme_e)));
        tracks.push_back(make_shared<Track>(length, speeds, type_chr, TrackDirection::WESTBOUND, make_pair(extreme_e, extreme_w)));
    }
    
    int global_train_id = 0;
    BOOST_FOREACH(const ptree::value_type& tr_child, pt.get_child("trains")) {
        char tr_class = tr_child.second.get<char>("class");
        int entry_time = tr_child.second.get<int>("entry_time");
        std::shared_ptr<Junction> origin_junct = junction_by_id(tr_child.second.get<int>("origin_node"));
        std::shared_ptr<Junction> destination_junct = junction_by_id(tr_child.second.get<int>("destination_node"));
        float speed_multi = tr_child.second.get<float>("speed_multi");
        float length = tr_child.second.get<float>("length");
        float tob = tr_child.second.get<float>("tob");
        bool hazmat = tr_child.second.get<bool>("hazmat");
        int initial_delay = tr_child.second.get<int>("initial_delay");
        int terminal_wt = tr_child.second.get<int>("terminal_wt");
        
        Schedule schedule;
        BOOST_FOREACH(const ptree::value_type& sc_child, tr_child.second.get_child("schedule")) {
            std::shared_ptr<Junction> junct = junction_by_id(sc_child.second.get<int>("node"));
            int time_in = sc_child.second.get<int>("time");
            
            schedule.push_back(make_pair(junct, time_in));
        }
        
        trains.push_back(make_shared<Train>(global_train_id++, tr_class, entry_time, origin_junct, destination_junct, speed_multi, length, tob, hazmat, initial_delay, schedule, terminal_wt));
    }
    
    BOOST_FOREACH(const ptree::value_type& mow_child, pt.get_child("mow")) {
        std::shared_ptr<Junction> extreme1 = junction_by_id(mow_child.second.get<int>("extreme1"));
        std::shared_ptr<Junction> extreme2 = junction_by_id(mow_child.second.get<int>("extreme2"));
        int start_time = mow_child.second.get<int>("start_time");
        int end_time = mow_child.second.get<int>("end_time");
        
        mow.push_back(make_tuple(extreme1, extreme2, start_time, end_time));
    }
}

void Data::print() const {
    cout << "Number of trains: " << num_trains << endl;
    cout << "Number of junctions: " << num_junctions << endl;
    cout << "Number of time intervals: " << num_times << endl;
    cout << "Speeds: " << speed_we << ", " << speed_we << ", " << speed_siding << ", " << speed_switch << ", " << speed_xover << endl;
    cout << "Tracks: ";
    for(const std::shared_ptr<Track>& t : tracks) {
        cout << *t << " ";
    }
    cout << endl;
    cout << "Trains: ";
    for(const std::shared_ptr<Train>& t : trains) {
        cout << *t << " ";
    }
    cout << endl;
}

std::shared_ptr<Junction> Data::junction_by_id(const int id) const {
    auto junct_it = find_if(junctions.begin(), junctions.end(),
    [&id] (const std::shared_ptr<Junction>& j) {
        return j->id == id;
    });
    if(junct_it == junctions.end()) {
        return nullptr;
    } else {
        return *junct_it;
    }
}

#endif