#ifndef FEUP_CAL_PROJ_KOSARAJU_H
#define FEUP_CAL_PROJ_KOSARAJU_H

#include <Graph.h>
#include <Connectivity/SCCAlgorithm.h>
#include <stack>

using namespace std;

class Kosaraju : public SCCAlgorithm {
public:
    Kosaraju(Graph * graph_);

    void execute() override;

private:

    Graph reversedGraph;

    unordered_map<Node, bool> vis1;

    unordered_map<Node, bool> vis2;

    stack<Node> st;

    void dfsOnGraph(Node u);

    void dfsOnReversedGraph(Node u, Node  rep);
};



#endif //FEUP_CAL_PROJ_KOSARAJU_H
