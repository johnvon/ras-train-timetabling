#ifndef DATA_H
#define DATA_H

#include <network/segment.h>
#include <network/train.h>
#include <network/mow.h>

#include <memory>
#include <string>
#include <vector>

struct Data {
    std::string file_name;
    std::string instance_name;
    double speed_ew, speed_we, speed_siding, speed_switch, speed_xover;
    int time_intervals, want_time_tw_start, want_time_tw_end, schedule_tw_end, headway;
    std::unordered_map<char, double> general_delay_price;
    double terminal_delay_price, schedule_delay_price, unpreferred_price;
    int segments_number, trains_number;
    std::vector<std::shared_ptr<const Segment>> segments;
    std::vector<Train> trains;
    std::vector<Mow> mow;
    
    Data(const std::string file_name);
    
    void parse();
    void print_stats() const;
    bool is_main(std::shared_ptr<const Segment> m, std::shared_ptr<const Segment> s) const;
    int segment_id_for_node(int node_number, bool westbound_train) const;
};

#endif