#ifndef SEGMENT_H
#define SEGMENT_H

struct Segment {
    int id;
    int extreme_1;
    int extreme_2;
    double min_distance_from_w;
    double min_distance_from_e;
    char type;
    double length;
    bool eastbound;
    bool westbound;
    
    Segment(const int id, const int extreme_1, const int extreme_2, const double min_distance_from_w, const double min_distance_from_e, const char type, const double length, const bool eastbound, const bool westbound) : id{id}, extreme_1{extreme_1}, extreme_2{extreme_2}, min_distance_from_w{min_distance_from_w}, min_distance_from_e{min_distance_from_e}, type{type}, length{length}, eastbound{eastbound}, westbound{westbound} {}
};

#endif