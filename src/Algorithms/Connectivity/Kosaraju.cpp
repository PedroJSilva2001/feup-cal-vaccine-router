#include "Connectivity/Kosaraju.h"

Kosaraju::Kosaraju(Graph * graph_) : SCCAlgorithm(graph_) {
    reversedGraph = graph_->getReversedGraph();
    st = stack<Node>();
    for (Node u : graph->getNodeSet()) {
        vis1.insert(make_pair(u,false));
        vis2.insert(make_pair(u,false));
        scc.insert(make_pair(u, NULL_NODE));
    }
}

void Kosaraju::execute() {
    auto start_time = hrc::now();
    for (Node u : graph->getNodeSet()) {
        if (!vis1.at(u)) {
            dfsOnGraph(u);
        }
    }
    while(!st.empty()) {
        Node u = st.top();
        st.pop();
        if (!vis2.at(u)) {
            dfsOnReversedGraph(u, u);
        }
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

void Kosaraju::dfsOnGraph(Node u) {
    vis1.at(u) = true;
    for (Edge e : graph->getOutgoingEdges(u)) {
        if (!vis1.at(e.dest)) {
            dfsOnGraph(e.dest);
        }
    }
    st.push(u);
}

void Kosaraju::dfsOnReversedGraph(Node u, Node rep) {
    vis2.at(u) = true;
    scc.at(u) = rep;
    for (Edge e : reversedGraph.getOutgoingEdges(u)) {
        if (!vis2.at(e.dest)) {
            dfsOnReversedGraph(e.dest, rep);
        }
    }
}
