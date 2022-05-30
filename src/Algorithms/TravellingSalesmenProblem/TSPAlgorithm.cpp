#include "TSPAlgorithm.h"

TSPAlgorithm::TSPAlgorithm(const unordered_set<Node> & V_, Node s_, CostFunctionTSP *cost_) {
    V = V_;
    s = s_;
    cost = cost_;
    route = vector<Node>();
}

const vector<Node> &TSPAlgorithm::getRoute() const {
    return route;
}

int TSPAlgorithm::getExecutionTime() const {
    return executionTime;
}
