#include "Astar.h"
#include <iostream>
#include <queue>
Astar::Astar(MutablePriorityQueue<NodeWrapper> priorityQueue_, Graph *graph_, AstarHeuristic * heuristic_, Node s_,
             Node t_) : ShortestPathAlgorithm(graph_, s_, t_) {
    priorityQueue = std::move(priorityQueue_);
    graph = graph_;
    heuristic = heuristic_;

    for(Node u : graph->getNodeSet()) {
        NodeWrapper * wp = new NodeWrapper(u, u == s ? (*heuristic)(s) : INF);
        dist.insert(make_pair(u, u == s ? 0 : INF));
        hdist.insert(make_pair(u, u == s ? (*heuristic)(s) : INF));
        prev.insert(make_pair(u, NULL_NODE));
        priorityQueue.insert(wp);
        wrapper.insert(make_pair(u, wp));
    }
}

void Astar::execute() {
    auto start_time = hrc::now();
    Weight newDist;
    while(!priorityQueue.empty()) {
        Node u = priorityQueue.extractMin()->node;
        nodesAnalyzed++;
        if (u == t) {
            break;
        }
        for (Edge e : graph->getOutgoingEdges(u)) {
            newDist = dist.at(u) + e.weight;
            if (dist.at(e.dest) > newDist) {
                hdist.at(e.dest) = newDist + (*heuristic)(e.dest);
                dist.at(e.dest) = newDist;
                prev.at(e.dest) = u;
                wrapper.at(e.dest)->dist = hdist.at(e.dest);
                priorityQueue.decreaseKey(wrapper.at(e.dest));
            }
        }
    }
    totalWeight = dist.at(t);
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

