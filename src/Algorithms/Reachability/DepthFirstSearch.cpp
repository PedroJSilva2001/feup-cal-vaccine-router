#include "DepthFirstSearch.h"
#include <iostream>

DepthFirstSearch::DepthFirstSearch(Graph * graph_, Node s_, const unordered_set<Node> & poi_) {
    graph = graph_;
    s = s_;
    poi = poi_;
    reachable = vector<Node>();
    vis = unordered_map<Node, bool>();
    for (Node u : graph->getNodeSet()) {
        vis.insert(make_pair(u, false));
        vis.at(u) = false;
    }
}

void DepthFirstSearch::execute() {
    auto start_time = hrc::now();
    dfsVisit(s);
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

const vector<Node> &DepthFirstSearch::getReachable() const {
    return reachable;
}

void DepthFirstSearch::dfsVisit(Node u) {
    vis.at(u) = true;
    if (poi.find(u) != poi.end() && u != s) {
        reachable.push_back(u);
    }
    for (Edge e : graph->getOutgoingEdges(u)) {
        if (!vis.at(e.dest)) {
            dfsVisit(e.dest);
        }
    }
}

int DepthFirstSearch::getExecutionTime() const {
    return executionTime;
}
