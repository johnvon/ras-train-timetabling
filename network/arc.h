#ifndef ARC_H
#define ARC_H

#include <memory>

#include <network/track.h>

class Arc {
public:
    int                      id;
    std::shared_ptr<Track>   track;
};

#endif