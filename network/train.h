#ifndef TRAIN_H
#define TRAIN_H

#include <memory>
#include <vector>
using std::vector;
#include <utility>
using std::pair;
#include <iostream>
using std::ostream;

#include <network/track.h>

enum class TrainPrecedenceType : char { A = 'A', B = 'B', C = 'C', D = 'D', E = 'E', F = 'F' };
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
    
    Train(const int id, const TrainPrecedenceType pc, const TrainAdherenceType ac, const int entry_time, const std::shared_ptr<Junction> origin, const std::shared_ptr<Junction> destination, const TrackDirection direction, const float sm, const float length, const float tob, const bool hazmat, const int ide, const Schedule schedule, const int tw) : id(id), precedence_class(pc), adherence_class(ac), entry_time(entry_time), origin(origin), destination(destination), direction(direction), speed_multiplier(sm), length(length), tob(tob), hazmat(hazmat), initial_delay(ide), schedule(schedule), terminal_wt(tw) {}
    Train(const int id, const char pc, const int entry_time, const std::shared_ptr<Junction> origin, const std::shared_ptr<Junction> destination, const float sm, const float length, const float tob, const bool hazmat, const int ide, const Schedule schedule, const int tw);
    
    // Returns true if the train is of schedule adherence type or false otherwise
    bool is_sa() const { return (adherence_class == TrainAdherenceType::SCHEDULE_ADHERENCE); }
    
    // Checks that the data is congruent
    void check_valid() const;
};

ostream& operator<<(ostream& out, const Train& t);

#endif