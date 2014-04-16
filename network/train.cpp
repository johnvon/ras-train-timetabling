#ifndef TRAIN_CPP
#define TRAIN_CPP

#include <cassert>

#include <network/train.h>

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

#endif