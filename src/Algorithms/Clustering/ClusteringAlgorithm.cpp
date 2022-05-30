#include "ClusteringAlgorithm.h"

ClusteringAlgorithm::ClusteringAlgorithm(const unordered_set<Node> &depots_, const unordered_set<Node> &clients_,
                                         const unordered_map<Node, unordered_map<Node, Weight>> &cost_, unordered_map<Node, Node> &scc_ ) {
    assignedClients = unordered_map<Node, unordered_set<Node>>();
    depots = depots_;
    clients = clients_;
    cost = cost_;
    scc = scc_;
    for (Node u : depots) {
        assignedClients.insert(make_pair(u, unordered_set<Node>()));
    }
}

const unordered_map<Node, unordered_set<Node>> & ClusteringAlgorithm::getAssignedClients() const {
    return assignedClients;
}

int ClusteringAlgorithm::getExecutionTime() const {
    return executionTime;
}