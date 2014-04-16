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

enum class TrackType { MAIN0, MAIN1, MAIN2, SIDING, SWITCH, XOVER };
enum class TrackDirection { EASTBOUND, WESTBOUND };

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
    Track(const float length, const vector<float> max_speeds, const string type_str, const TrackDirection direction, const Extremes extremes);
};

#endif