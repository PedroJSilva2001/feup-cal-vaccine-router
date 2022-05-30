#ifndef FEUP_CAL_PROJ_DIJKSTRA_H
#define FEUP_CAL_PROJ_DIJKSTRA_H

#include <MutablePriorityQueue.h>
#include <NodeWrapper.h>
#include <ShortestPathAlgorithm.h>

class Dijkstra : public ShortestPathAlgorithm {
public:
    Dijkstra(MutablePriorityQueue<NodeWrapper> priorityQueue_, Graph * graph_, Node s_, Node t_);

    void execute() override;

    const unordered_map<Node, Weight> & getDist() const;

private:
    MutablePriorityQueue<NodeWrapper> priorityQueue;

    unordered_map<Node, Weight> dist;

    unordered_map<Node, NodeWrapper*> wrapper;
};


#endif //FEUP_CAL_PROJ_DIJKSTRA_H
