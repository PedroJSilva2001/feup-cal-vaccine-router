#include "ShortestPathAlgorithm.h"

ShortestPathAlgorithm::ShortestPathAlgorithm(Graph *graph_, Node s_, Node t_) {
    graph = graph_;
    s = s_;
    t = t_;
    totalWeight = INF;
    prev = unordered_map<Node, Node>();
    executionTime = 0;
    nodesAnalyzed = 0;
}

const Weight &ShortestPathAlgorithm::getTotalWeight() const {
    return totalWeight;
}

const unordered_map<Node, Node> &ShortestPathAlgorithm::getPrev() const {
    return prev;
}

int ShortestPathAlgorithm::getExecutionTime()  const {
    return executionTime;
}

unsigned ShortestPathAlgorithm::getNodesAnalyzed() const {
    return nodesAnalyzed;
}