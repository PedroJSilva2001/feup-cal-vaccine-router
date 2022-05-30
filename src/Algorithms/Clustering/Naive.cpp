#include "Naive.h"

Naive::Naive(const unordered_set<Node> &depots_, const unordered_set<Node> &clients_,
             const unordered_map<Node, unordered_map<Node, Weight>> &cost_, unordered_map<Node, Node> &scc_) :
             ClusteringAlgorithm(depots_, clients_, cost_, scc_) {

}

void Naive::execute() {
    auto start_time = hrc::now();
    unordered_map<Node, Node> depotAssigned = unordered_map<Node, Node>();
    unordered_set<Node> unassigned(clients);
    for (Node client : clients) {
        depotAssigned.insert(make_pair(client, NULL_NODE));
    }
    Weight currDist;
    for (Node client : clients) {
        currDist = INF;
        for (Node depot : depots) {
            if (currDist > cost.at(depot).at(client) && scc.at(depot) == scc.at(client)) {
                currDist = cost.at(depot).at(client);
                depotAssigned.at(client) = depot;
            }
        }
    }
    for (auto pair : depotAssigned) {
        assignedClients.at(pair.second).insert(pair.first);
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

