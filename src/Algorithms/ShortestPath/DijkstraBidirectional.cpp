#include "DijkstraBidirectional.h"

#include <utility>

DijkstraBidirectional::DijkstraBidirectional(MutablePriorityQueue<NodeWrapper> priorityQueue_,
                                             MutablePriorityQueue<NodeWrapper> priorityQueueRev_, Graph *graph_,
                                             Node s_, Node t_) : ShortestPathAlgorithm(graph_, s_, t_){
    priorityQueueF = std::move(priorityQueue_);
    priorityQueueB = std::move(priorityQueueRev_);
    SF = unordered_set<Node>();
    SB = unordered_set<Node>();
    for(Node u : graph->getNodeSet()) {
        NodeWrapper * wp = new NodeWrapper(u, u == s ? 0.0 : INF);
        NodeWrapper * wp2 = new NodeWrapper(u, u == t ? 0.0 : INF);
        distF.insert(make_pair(u, u == s ? 0.0 : INF));
        distB.insert(make_pair(u, u == t ? 0.0 : INF));
        prevF.insert(make_pair(u, NULL_NODE));
        prevB.insert(make_pair(u, NULL_NODE));
        priorityQueueF.insert(wp);
        priorityQueueB.insert(wp2);
        wrapperF.insert(make_pair(u, wp));
        wrapperB.insert(make_pair(u, wp2));
    }
}

void DijkstraBidirectional::execute() {
    Node mid;
    Graph graphRev = graph->getReversedGraph();

    auto start_time = hrc::now();
    while (!priorityQueueF.empty() && !priorityQueueB.empty()) {
        Node u = priorityQueueF.extractMin()->node;
        Node v = priorityQueueB.extractMin()->node;
        nodesAnalyzed+=2;
        if (distF.at(u) + distB.at(v) >= totalWeight) {
            break;
        }
        if (SF.find(u) == SF.end()) {
            for (Edge e : graph->getOutgoingEdges(u)) {
                if (distF.at(e.dest) > distF.at(u) + e.weight) {
                    distF.at(e.dest) = distF.at(u) + e.weight;
                    prevF.at(e.dest) = u;
                    wrapperF.at(e.dest)->dist = distF.at(e.dest);
                    priorityQueueF.decreaseKey(wrapperF.at(e.dest));
                }
                if (SB.find(e.dest) != SB.end() && distF.at(u) + e.weight + distB.at(e.dest) < totalWeight) {
                    totalWeight = distF.at(u) + e.weight + distB.at(e.dest);
                    prevF.at(e.dest) = u;
                    mid = e.dest;
                }
            }
            SF.insert(u);
        }
        if (SB.find(v) == SB.end()) {
            for (Edge e : graphRev.getOutgoingEdges(v)) {
                if (distB.at(e.dest) > distB.at(v) + e.weight) {
                    distB.at(e.dest) = distB.at(v) + e.weight;
                    prevB.at(e.dest) = v;
                    wrapperB.at(e.dest)->dist = distB.at(e.dest);
                    priorityQueueB.decreaseKey(wrapperB.at(e.dest));
                }
                if (SF.find(e.dest) != SF.end() && distB.at(v) + e.weight + distF.at(e.dest) < totalWeight) {
                    totalWeight = distB.at(v) + e.weight + distF.at(e.dest);
                    prevF.at(v) = e.dest;
                    prevB.at(e.dest) = v;
                    mid = v;
                }
            }
            SB.insert(v);
        }
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
    setPrev(mid);
}

void DijkstraBidirectional::setPrev(Node mid) {
    if (totalWeight == INF) {  // or if dist[u] == INF || dist[v] == INF
        return;
    }
    Node curr = mid;
    while (prevB.at(curr) != NULL_NODE) {
        Node prB = prevB.at(curr);
        prevF.at(prB) = curr;
        curr = prevB.at(curr);
    }
    prev = prevF;
}
