#ifndef FEUP_CAL_PROJ_CLUSTERINGALGORITHM_H
#define FEUP_CAL_PROJ_CLUSTERINGALGORITHM_H

#include <vector>
#include <chrono>
#include <Graph.h>

typedef std::chrono::high_resolution_clock hrc;

class ClusteringAlgorithm {
public:
    ClusteringAlgorithm(const unordered_set<Node> & depots_, const unordered_set<Node> & clients_,
                        const unordered_map<Node, unordered_map<Node, Weight>> & cost_, unordered_map<Node, Node> &scc_);

    const unordered_map<Node, unordered_set<Node>> & getAssignedClients() const;

    virtual void execute() = 0;

    int getExecutionTime() const;

protected:
    unordered_set<Node> depots;

    unordered_set<Node> clients;

    unordered_map<Node, unordered_map<Node, Weight>> cost;

    unordered_map<Node, unordered_set<Node>> assignedClients;

    unordered_map<Node, Node> scc;

    int executionTime;
};


#endif //FEUP_CAL_PROJ_CLUSTERINGALGORITHM_H
