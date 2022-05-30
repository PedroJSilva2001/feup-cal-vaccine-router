#ifndef FEUP_CAL_PROJ_TSPALGORITHM_H
#define FEUP_CAL_PROJ_TSPALGORITHM_H

#include <Graph.h>
#include <vector>
#include <chrono>


typedef std::chrono::high_resolution_clock hrc;


class CostFunctionTSP {
public:
    virtual Weight operator()(const unordered_set<Node> & S, Node u, Node v) const = 0;
};

class TSPAlgorithm {
public:
    TSPAlgorithm(const unordered_set<Node> & V_, Node s_, CostFunctionTSP * cost_);

    virtual void execute() = 0;

    const vector<Node> & getRoute() const;

    int getExecutionTime() const;

protected:
    unordered_set<Node> V;

    Node s;

    CostFunctionTSP * cost;

    vector<Node> route;

    int executionTime;
};


#endif //FEUP_CAL_PROJ_TSPALGORITHM_H
