#ifndef PROBLEM_H
#define PROBLEM_H

#include <memory>
#include <unordered_map>
using std::unordered_map;

#include <network/graph.h>
#include <network/train.h>
#include <preprocessing/data.h>

typedef unordered_map<std::shared_ptr<Train>, std::shared_ptr<Graph>> GraphMap;

class Problem {
public:
    Data        data;
    GraphMap    graphs;
};

#endif