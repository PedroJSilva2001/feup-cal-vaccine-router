#include "CapacitatedNearestNeighbour.h"
#include <queue>
/*CapacitatedNearestNeighbour::CapacitatedNearestNeighbour(const unordered_set<Node> &V_, Node s_,
                                                         const unordered_map<Node, unordered_map<Node, Weight>> &cost_,
                                                         const unordered_map<Node, unsigned int> &demand_,
                                                         unsigned int vehicleNumber_, Weight t_, unsigned int C_)
        : VRPAlgorithm(V_, s_, cost_, demand_, vehicleNumber_, t_, C_) {
    unvisited = unordered_set<Node>();
    for (Node u : V) {
        if (u == s) {
            continue;
        }
        unvisited.insert(u);
    }

}*/
CapacitatedNearestNeighbour::CapacitatedNearestNeighbour(const unordered_set<Node> &clients_, Node depot_,
                                                         const unordered_map<Node, unordered_map<Node, Weight>> &cost_,
                                                         const unordered_map<Node, unsigned int> &demand_,
                                                         unsigned int vehicleNumber_, Weight t_, unsigned int C_)
        : VRPAlgorithm(clients_, depot_, cost_, demand_, t_, C_) {
    unvisited = unordered_set<Node>();
    for (Node u : clients_) {
        if (u == depot) {
            continue;
        }
        unvisited.insert(u);
    }

    vehicleNumber = vehicleNumber_;
}

void CapacitatedNearestNeighbour::execute() {
    auto start_time = hrc::now();

    unsigned m = vehicleNumber;
    vector<Node> route;
    Weight currTime;
    unsigned currCapacity;
    Node currNode;
    bool assignRoute;
    priority_queue<NodeWrapperNN> pq;
    while (m--) {
        route = vector<Node>();
        assignRoute = true;
        currTime = 0;
        currNode = depot;
        currCapacity = 0;
        if (currNode == depot) {
            route.push_back(depot);
            continue;
        }
        while (assignRoute) {
            pq = priority_queue<NodeWrapperNN>();
            for (Node u : unvisited) {
                pq.push(NodeWrapperNN(u, cost.at(currNode).at(u)));
            }
            while (!pq.empty()) {
                NodeWrapperNN wp = pq.top();
                pq.pop();
                if (currTime + wp.travelTime > t) {
                    assignRoute = false;
                    break;
                }
                if (currCapacity + demand.at(wp.node) <= C) {
                    route.push_back(wp.node);
                    assignRoute = true;
                    unvisited.erase(wp.node);
                    currTime += wp.travelTime;
                    currCapacity += demand.at(wp.node);
                    currNode = wp.node;
                }
                else {
                    assignRoute = false;
                }
            }
        }
        route.push_back(depot);
        //routes.push_back(route);
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();

}