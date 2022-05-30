#ifndef FEUP_CAL_PROJ_SHORTESTPATHALGORITHM_H
#define FEUP_CAL_PROJ_SHORTESTPATHALGORITHM_H

#include <NodeWrapper.h>
#include <MutablePriorityQueue.h>
#include <chrono>

typedef std::chrono::high_resolution_clock hrc;

class ShortestPathAlgorithm {
public:
    ShortestPathAlgorithm(Graph *graph_, Node s_, Node t_);

    const Weight & getTotalWeight() const;

    const unordered_map<Node, Node> & getPrev() const;

    virtual void execute() = 0;

    int getExecutionTime() const;

    unsigned getNodesAnalyzed() const;

protected:
    Graph * graph;

    Node s;

    Node t;

    Weight totalWeight;

    unordered_map<Node, Node> prev;

    int executionTime;

    unsigned nodesAnalyzed;
};


#endif //FEUP_CAL_PROJ_SHORTESTPATHALGORITHM_H
