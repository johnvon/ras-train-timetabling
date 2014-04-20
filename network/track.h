#ifndef TRACK_H
#define TRACK_H

#include <utility>
using std::pair;
#include <memory>
using std::make_shared;
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <network/junction.h>

enum class TrackType : char { MAIN0 = '0', MAIN1 = '1', MAIN2 = '2', SIDING = 'S', SWITCH = 'T', XOVER = 'X' };
enum class TrackDirection : char { EASTBOUND = 'E', WESTBOUND = 'W' };

typedef pair<std::shared_ptr<Junction>, std::shared_ptr<Junction>> Extremes;

class Track {
public:
    float           length;
    float           max_speed;
    TrackType       type;
    TrackDirection  direction;
    Extremes        extremes;
    
    Track(const float length, const float max_speed, const TrackType type, const TrackDirection direction, const Extremes extremes) : length(length), max_speed(max_speed), type(type), direction(direction), extremes(extremes) {}
    
    // max_speeds: 0 => we, 1 => ew, 2 => siding, 3 => switch, 4 => xover
    Track(const float length, const vector<float> max_speeds, const char type_chr, const TrackDirection direction, const Extremes extremes);
};

#endif