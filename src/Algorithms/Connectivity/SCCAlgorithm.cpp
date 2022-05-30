#include "SCCAlgorithm.h"

SCCAlgorithm::SCCAlgorithm(Graph *graph_) {
    graph = graph_;
    scc = unordered_map<Node, Node>();
}

const unordered_map<Node, Node> & SCCAlgorithm::getScc() const {
    return scc;
}

int SCCAlgorithm::getExecutionTime() const {
    return executionTime;
}
