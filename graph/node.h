#ifndef NODE_H
#define NODE_H

#include <network/segment.h>

#include <memory>
#include <stdexcept>
#include <string>

struct Node {
    std::shared_ptr<const Segment> s;
    int t;
    bool source;
    bool sink;
    
    Node() : s{nullptr}, t{-1}, source{false}, sink{false} {}
    Node(const std::shared_ptr<const Segment> s, const int t, const bool source = false, const bool sink = false) : s{s}, t{t}, source{source}, sink{sink} {}
    
    bool operator==(const Node& other) const {
        if(s == nullptr || other.s == nullptr) {
            return (source == other.source && sink == other.sink);
        }
        return (s->id == other.s->id && t == other.t);
    }
    bool operator!=(const Node& other) const {
        return !(*this == other);
    }
    std::string str() const {
        if(source) { return "sigma"; }
        if(sink) { return "tau"; }
        if(s != nullptr) { return (std::to_string(s->id) + "_" + std::to_string(t)); }
        throw std::runtime_error("Such a node should not exist!");
    }
};

#endif