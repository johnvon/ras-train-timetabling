#ifndef TRACK_CPP
#define TRACK_CPP

#include <stdexcept>
using std::runtime_error;

#include <network/track.h>

Track::Track(const float length, const vector<float> max_speeds, const char type_chr, const TrackDirection direction, const Extremes extremes) : length(length), direction(direction), extremes(extremes) {
    type = static_cast<TrackType>(type_chr);
    
    switch(type_chr) {
        case '0':
            type = TrackType::MAIN0;
            max_speed = (direction == TrackDirection::EASTBOUND ? max_speeds[0] : max_speeds[1]);
            break;
        case '1':
            type = TrackType::MAIN1;
            max_speed = (direction == TrackDirection::EASTBOUND ? max_speeds[0] : max_speeds[1]);
            break;
        case '2':
            type = TrackType::MAIN2;
            max_speed = (direction == TrackDirection::EASTBOUND ? max_speeds[0] : max_speeds[1]);
            break;
        case 'T':
            type = TrackType::SWITCH;
            max_speed = max_speeds[3];
            break;
        case 'S':
            type = TrackType::SIDING;
            max_speed = max_speeds[2];
            break;
        case 'X':
            type = TrackType::XOVER;
            max_speed = max_speeds[4];
            break;
        default:
            throw runtime_error("Unrecognised track type: " + type_chr);
    }
}

ostream& operator<<(ostream& out, const Track& t) {
    out << "[" << t.extremes.first->id << ", " << t.extremes.second->id << "]";
    return out;
}

#endif