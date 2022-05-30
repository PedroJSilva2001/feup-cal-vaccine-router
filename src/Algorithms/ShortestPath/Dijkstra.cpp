#include "Dijkstra.h"

#include <utility>

Dijkstra::Dijkstra(MutablePriorityQueue<NodeWrapper> priorityQueue_, Graph *graph_, Node s_, Node t_) :
    ShortestPathAlgorithm(graph_, s_, t_) {
    priorityQueue = std::move(priorityQueue_);
    for(Node u : graph->getNodeSet()) {
        NodeWrapper * wp = new NodeWrapper(u, u == s ? 0 : INF);
        dist.insert(make_pair(u, u == s ? 0 : INF));
        prev.insert(make_pair(u, NULL_NODE));
        priorityQueue.insert(wp);
        wrapper.insert(make_pair(u, wp));
    }
}

void Dijkstra::execute() {
    auto start_time = hrc::now();
    Weight newDist;
    while(!priorityQueue.empty()) {
        Node u = priorityQueue.extractMin()->node;
        nodesAnalyzed++;
        if (u == t && t != NULL_NODE) {
            break;
        }
        for (Edge e : graph->getOutgoingEdges(u)) {
            newDist = dist.at(u) + e.weight;
            if (dist.at(e.dest) > newDist) {
                wrapper.at(e.dest)->dist = newDist;
                dist.at(e.dest) = newDist;
                prev.at(e.dest) = u;
                priorityQueue.decreaseKey(wrapper.at(e.dest));
            }
        }
    }
    if (t != NULL_NODE) {
        totalWeight = dist.at(t);
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

const unordered_map<Node, Weight> &Dijkstra::getDist() const {
    return dist;
}