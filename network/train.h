#ifndef TRAIN_H
#define TRAIN_H

#include <memory>
#include <vector>
using std::vector;
#include <utility>
using std::pair;

#include <network/track.h>

enum class TrainPrecedenceType : int { A, B, C, D, E, F };
enum class TrainAdherenceType { SCHEDULE_ADHERENCE, NON_SCHEDULE_ADHERENCE };

typedef vector<pair<std::shared_ptr<Junction>, int>> Schedule;

class Train {
public:
    int                         id;
    TrainPrecedenceType         precedence_class;
    TrainAdherenceType          adherence_class;
    int                         entry_time;
    std::shared_ptr<Junction>   origin;
    std::shared_ptr<Junction>   destination;
    TrackDirection              direction;
    float                       speed_multiplier;
    float                       length;
    float                       tob;
    bool                        hazmat;
    int                         initial_delay;
    Schedule                    schedule;
    int                         terminal_wt; // Want time
    
    // Returns true if the train is of schedule adherence type or false otherwise
    bool is_sa() const { return (adherence_class == TrainAdherenceType::SCHEDULE_ADHERENCE); }
    
    // Checks that the data is congruent
    void check_valid() const;
};

#endif