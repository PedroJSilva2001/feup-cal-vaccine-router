#include "BuildVRP.h"

BuildVRP::BuildVRP(const vector<Node> &fullRoute_, const unordered_set<Node> &clients_, Node depot_,
                   const unordered_map<Node, unordered_map<Node, Weight>> &cost_,
                   const unordered_map<Node, unsigned int> &demand_, Weight t_, unsigned int C_) :
                   VRPAlgorithm(clients_, depot_, cost_, demand_, t_, C_){
    fullRoute = fullRoute_;
}

void BuildVRP::execute() {
    auto start_time = hrc::now();
    unsigned id = 0;
    Route * route = new Route(id++);
    Weight currCost = 0;
    unsigned currCap = 0;
    Node curr = depot;
    route->insert(depot, cost);
    for (unsigned i = 1; i < fullRoute.size(); i++) {
        Node u = fullRoute.at(i);
        // if current node is the last client in the cycly since the time limit is taken into account
        // when the vehicle goes back to the depot this can be done
        if (i == fullRoute.size()-2) {
            route->insert(u, cost.at(curr).at(u), demand.at(u));
            route->insert(depot, cost.at(u).at(depot), 0);
            routes.push_back(route);
            break;
        }
        if (currCost + cost.at(curr).at(u) > t || currCap + demand.at(u) > C) {
            currCap = demand.at(u);
            currCost = cost.at(curr).at(u);
            //Go back to depot
            route->insert(depot, cost.at(curr).at(u), 0);
            routes.push_back(route);
            //New route is created
            route = new Route(id++);
            route->insert(depot, 0, 0);
        }
        else {
            currCap += demand.at(u);
            currCost += cost.at(curr).at(u);
        }
        route->insert(u, cost.at(curr).at(u), demand.at(u));
        curr = u;
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}