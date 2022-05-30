#ifndef FEUP_CAL_PROJ_ASTAR_H
#define FEUP_CAL_PROJ_ASTAR_H

#include <NodeWrapper.h>
#include <MutablePriorityQueue.h>
#include <ShortestPathAlgorithm.h>

class AstarHeuristic {
public:
    virtual Weight operator()(Node u) const = 0;
};

class Astar : public ShortestPathAlgorithm {
public:
    Astar(MutablePriorityQueue<NodeWrapper> priorityQueue_, Graph *graph_, AstarHeuristic *heuristic_, Node s_, Node t_);

    void execute() override;

private:
    MutablePriorityQueue<NodeWrapper> priorityQueue;

    AstarHeuristic * heuristic;

    unordered_map<Node, Weight> dist;

    unordered_map<Node, Weight> hdist;

    unordered_map<Node, NodeWrapper*> wrapper;
};


#endif //FEUP_CAL_PROJ_ASTAR_H
