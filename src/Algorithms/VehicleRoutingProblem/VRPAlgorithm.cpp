#include "VRPAlgorithm.h"

VRPAlgorithm::VRPAlgorithm(const unordered_set<Node> & clients_, Node depot_,
                           const unordered_map<Node, unordered_map<Node, Weight>> & cost_,
                           const unordered_map<Node, unsigned int> & demand_, Weight t_,
                           unsigned C_) : t(t_) , C(C_) {
    clients = clients_;
    depot = depot_;
    cost = cost_;
    demand = demand_;
    routes = vector<Route*>();
}

const vector<Route*> &VRPAlgorithm::getRoutes() const {
    return routes;
}

int VRPAlgorithm::getExecutionTime() const {
    return executionTime;
}
