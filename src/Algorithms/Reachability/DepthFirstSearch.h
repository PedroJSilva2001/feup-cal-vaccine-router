#ifndef FEUP_CAL_PROJ_DEPTHFIRSTSEARCH_H
#define FEUP_CAL_PROJ_DEPTHFIRSTSEARCH_H

#include <Graph.h>
#include <vector>
#include <chrono>


typedef std::chrono::high_resolution_clock hrc;


class DepthFirstSearch {
public:
    DepthFirstSearch(Graph * graph_, Node s_, const unordered_set<Node> & poi_);

    void execute();

    const vector<Node> & getReachable() const;

    int getExecutionTime() const;


private:
    Graph * graph;

    unordered_set<Node> poi;

    Node s;

    vector<Node> reachable;

    unordered_map<Node, bool> vis;

    void dfsVisit(Node u);

    int executionTime;
};


#endif //FEUP_CAL_PROJ_DEPTHFIRSTSEARCH_H
