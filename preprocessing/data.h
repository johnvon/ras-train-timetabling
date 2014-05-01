#ifndef DATA_H
#define DATA_H

#include <memory>
#include <vector>
using std::vector;
#include <string>
using std::string;
#include <tuple>

#include <network/train.h>
#include <network/junction.h>
#include <network/track.h>

typedef std::tuple<std::shared_ptr<Junction>, std::shared_ptr<Junction>, int, int> Mow;

class Data {
public:
    int                                 num_trains;
    int                                 num_junctions;
    int                                 num_times;
    vector<std::shared_ptr<Train>>      trains;
    vector<std::shared_ptr<Junction>>   junctions;
    vector<std::shared_ptr<Track>>      tracks;
    
    float                               speed_ew;
    float                               speed_we;
    float                               speed_siding;
    float                               speed_switch;
    float                               speed_xover;
    
    vector<Mow>                         mow;
    
    Data(const string data_file_name = "data/problem_data.json");
    
    void print() const;
    
private:
    std::shared_ptr<Junction> junction_by_id(const int id) const;
};

#endif