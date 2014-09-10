#ifndef TRAIN_H
#define TRAIN_H

#include <unordered_map>

typedef std::unordered_map<int, int> Schedule;

struct Train {
    int id;
    char cl;
    bool schedule_adherence;
    int entry_time;
    int origin_node;
    int destination_node;
    bool eastbound;
    bool westbound;
    double speed_multi;
    double length;
    int tob;
    bool heavy;
    bool hazmat;
    int terminal_wt;
    Schedule schedule;
    
    Train(const int id, const char cl, const bool schedule_adherence, const int entry_time, const int origin_node, const int destination_node, const bool eastbound, const bool westbound, const double speed_multi, const double length, const int tob, const bool hazmat, const int terminal_wt, const Schedule schedule) : id{id}, cl{cl}, schedule_adherence{schedule_adherence}, entry_time{entry_time}, origin_node{origin_node}, destination_node{destination_node}, eastbound{eastbound}, westbound{westbound}, speed_multi{speed_multi}, length{length}, tob{tob}, hazmat{hazmat}, terminal_wt{terminal_wt}, schedule{schedule} { heavy = (tob > 100); }    
};

#endif