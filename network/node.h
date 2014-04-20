#ifndef NODE_H
#define NODE_H

#include <memory>

#include <network/junction.h>

class Node {
public:
    std::shared_ptr<Junction>   junction;
    int                         time_interval;
    
    Node(std::shared_ptr<Junction> junction, const int time_interval) : junction(junction), time_interval(time_interval) {}
};

#endif