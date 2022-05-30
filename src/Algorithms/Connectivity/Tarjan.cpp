#include "Tarjan.h"

Tarjan::Tarjan(Graph *graph_) : SCCAlgorithm(graph_){
    st = stack<Node>();
    n = 0;
    for (Node u : graph->getNodeSet()) {
        scc.insert(make_pair(u, NULL_NODE));
        onStack.insert(make_pair(u, false));
        index.insert(make_pair(u, -1));
        lowlink.insert(make_pair(u, -1));
    }
}

void Tarjan::dfsToConnect(Node u) {
    index.at(u) = n;
    lowlink.at(u) = n;
    onStack.at(u) = true;
    n++;
    st.push(u);
    for (Edge e : graph->getOutgoingEdges(u)) {
        if (index.at(e.dest) == -1) {
            dfsToConnect(e.dest);
            lowlink.at(u) = min(lowlink.at(u), lowlink.at(e.dest));
        }
        else if (onStack.at(e.dest)) {
            lowlink.at(u) = min(lowlink.at(u), index.at(e.dest));
        }
    }
    if (lowlink.at(u) == index.at(u)) {
        while (!st.empty()) {
            Node v = st.top();
            st.pop();
            onStack.at(v) = false;
            scc.at(v) = u;
            if (u == v) {
                break;
            }
        }
    }
}

void Tarjan::execute() {
    auto start_time = hrc::now();

    for (Node u : graph->getNodeSet()) {
        if (index.at(u) == -1) {
            dfsToConnect(u);
        }
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();

}

unordered_map<Node, Node> & Tarjan::getScc() {
    return scc;
}
