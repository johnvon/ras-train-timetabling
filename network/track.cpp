#ifndef TRACK_CPP
#define TRACK_CPP

#include <stdexcept>
using std::runtime_error;

#include <network/track.h>

Track::Track(const float length, const vector<float> max_speeds, const string type_str, const TrackDirection direction, const Extremes extremes) : length(length), direction(direction), extremes(extremes) {
    if(type_str == "Main 0") {
        type = TrackType::MAIN0;
        max_speed = (direction == TrackDirection::EASTBOUND ? max_speeds[0] : max_speeds[1]);
    } else if(type_str == "Main 1") {
        type = TrackType::MAIN1;
        max_speed = (direction == TrackDirection::EASTBOUND ? max_speeds[0] : max_speeds[1]);
    } else if(type_str == "Main 2") {
        type = TrackType::MAIN2;
        max_speed = (direction == TrackDirection::EASTBOUND ? max_speeds[0] : max_speeds[1]);
    } else if(type_str == "Switch") {
        type = TrackType::SWITCH;
        max_speed = max_speeds[3];
    } else if(type_str == "Siding") {
        type = TrackType::SIDING;
        max_speed = max_speeds[2];
    } else if(type_str == "Xover") {
        type = TrackType::XOVER;
        max_speed = max_speeds[4];
    } else {
        throw runtime_error("Unrecognised track type: " + type_str);
    }
}

#endif