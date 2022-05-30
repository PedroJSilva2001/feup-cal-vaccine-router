#ifndef FEUP_CAL_PROJ_VRPALGORITHM_H
#define FEUP_CAL_PROJ_VRPALGORITHM_H

#include <Graph.h>
#include <vector>
#include <Route.h>
#include <chrono>

typedef std::chrono::high_resolution_clock hrc;


class VRPAlgorithm {
public:
    VRPAlgorithm(const unordered_set<Node> & clients_, Node depot_, const unordered_map<Node, unordered_map<Node, Weight>> & cost_,
                 const unordered_map<Node, unsigned> & demand_, Weight t_, unsigned C_);

    virtual void execute() = 0;

    const vector<Route*> & getRoutes() const;

    int getExecutionTime() const;


protected:
    Node depot;

    unordered_set<Node> clients;

    unordered_map<Node, unordered_map<Node, Weight>> cost;

    unordered_map<Node, unsigned> demand;

    const Weight t;

    const unsigned C;

    vector<Route*> routes;

    int executionTime;

};


#endif //FEUP_CAL_PROJ_VRPALGORITHM_H
