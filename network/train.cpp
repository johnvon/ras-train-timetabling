#ifndef TRAIN_CPP
#define TRAIN_CPP

#include <cassert>

#include <network/train.h>

Train::Train(const int id, const char pc, const int entry_time, const std::shared_ptr<Junction> origin, const std::shared_ptr<Junction> destination, const float sm, const float length, const float tob, const bool hazmat, const int ide, const Schedule schedule, const int tw) : id(id), entry_time(entry_time), origin(origin), destination(destination), speed_multiplier(sm), length(length), tob(tob), hazmat(hazmat), initial_delay(ide), schedule(schedule), terminal_wt(tw) {
    precedence_class = static_cast<TrainPrecedenceType>(pc);
    
    if(schedule.empty()) {
        adherence_class = TrainAdherenceType::NON_SCHEDULE_ADHERENCE;
    } else {
        adherence_class = TrainAdherenceType::SCHEDULE_ADHERENCE;
    }
    
    if(origin->terminal_type == TerminalType::EAST) {
        direction = TrackDirection::WESTBOUND;
    } else {
        direction = TrackDirection::EASTBOUND;
    }
}

void Train::check_valid() const {
    assert(origin->is_terminal);
    assert(destination->is_terminal);
    
    if(direction == TrackDirection::EASTBOUND) {
        assert(origin->terminal_type == TerminalType::WEST);
        assert(destination->terminal_type == TerminalType::EAST);
    } else {
        assert(origin->terminal_type == TerminalType::EAST);
        assert(destination->terminal_type == TerminalType::WEST);
    }
    
    assert(terminal_wt > entry_time);
}

ostream& operator<<(ostream& out, const Train& t) {
    out << "[id: " << t.id << ", class: " << static_cast<char>(t.precedence_class) << ", dir: " << static_cast<char>(t.direction) << "]";
    return out;
}

#endif