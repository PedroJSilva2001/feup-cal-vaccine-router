#include "NearestNeighbour.h"

NearestNeighbour::NearestNeighbour(const unordered_set<Node> & V_, Node s_, CostFunctionTSP *cost_) : TSPAlgorithm(V_, s_, cost_) {
    visited = unordered_set<Node>();
    route.push_back(s);
}

void NearestNeighbour::execute() {
    auto start_time = hrc::now();
    Node curr = s;
    Weight currCost;
    Weight newCost;
    for (long long i = 0; i < (long long)V.size() - 1; ++i) {
        visited.insert(curr);
        currCost = INF;
        for (Node u : V) {
            if (visited.find(u) == visited.end()) {
                newCost = (*cost)(visited, curr, u);
                if (currCost > newCost) {
                    currCost = newCost;
                    curr = u;
                }
            }
        }
        route.push_back(curr);
    }
    route.push_back(s);
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}
