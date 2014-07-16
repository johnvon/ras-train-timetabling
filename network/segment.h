#ifndef SEGMENT_H
#define SEGMENT_H

struct Segment {
    int id;
    int extreme_1;
    int extreme_2;
    char type;
    double length;
    bool eastbound;
    bool westbound;
    
    Segment(const int id, const int extreme_1, const int extreme_2, const char type, const double length, const bool eastbound, const bool westbound) : id{id}, extreme_1{extreme_1}, extreme_2{extreme_2}, type{type}, length{length}, eastbound{eastbound}, westbound{westbound} {}
};

#endif