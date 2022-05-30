#ifndef FEUP_CAL_PROJ_DIJKSTRABIDIRECTIONAL_H
#define FEUP_CAL_PROJ_DIJKSTRABIDIRECTIONAL_H

#include <NodeWrapper.h>
#include <MutablePriorityQueue.h>
#include <ShortestPathAlgorithm.h>

class DijkstraBidirectional : public ShortestPathAlgorithm {
public:
    DijkstraBidirectional(MutablePriorityQueue<NodeWrapper> priorityQueue_, MutablePriorityQueue<NodeWrapper> priorityQueueRev_,
                          Graph * graph_, Node s_, Node t_);

    void execute() override;

private:
    MutablePriorityQueue<NodeWrapper> priorityQueueF;

    MutablePriorityQueue<NodeWrapper> priorityQueueB;

    unordered_map<Node, Weight> distF;

    unordered_map<Node, Weight> distB;

    unordered_map<Node, Node> prevF;

    unordered_map<Node, Node> prevB;

    unordered_map<Node, NodeWrapper*> wrapperF;

    unordered_map<Node, NodeWrapper*> wrapperB;

    unordered_set<Node> SF;

    unordered_set<Node> SB;

    void setPrev(Node mid);
};


#endif //FEUP_CAL_PROJ_DIJKSTRABIDIRECTIONAL_H
