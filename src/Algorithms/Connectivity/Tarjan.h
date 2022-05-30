#ifndef FEUP_CAL_PROJ_TARJAN_H
#define FEUP_CAL_PROJ_TARJAN_H

#include <Graph.h>
#include <stack>
#include <Connectivity/SCCAlgorithm.h>
using namespace std;

class Tarjan : public SCCAlgorithm {
public:
    Tarjan(Graph * graph_);

    void execute() override;

    unordered_map<Node, Node> & getScc();

private:
    stack<Node> st;

    long long n;

    unordered_map<Node, bool> onStack;

    unordered_map<Node, long long> lowlink;

    unordered_map<Node, long long> index;

    void dfsToConnect(Node u);
};


#endif //FEUP_CAL_PROJ_TARJAN_H
