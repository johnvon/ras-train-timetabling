#ifndef MOW_H
#define MOW_H

struct Mow {
    int extreme_1;
    int extreme_2;
    int start_time;
    int end_time;
    
    Mow(const int extreme_1, const int extreme_2, const int start_time, const int end_time) : extreme_1{extreme_1}, extreme_2{extreme_2}, start_time{start_time}, end_time{end_time} {}
};

#endif