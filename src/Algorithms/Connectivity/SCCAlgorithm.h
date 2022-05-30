#ifndef FEUP_CAL_PROJ_SCCALGORITHM_H
#define FEUP_CAL_PROJ_SCCALGORITHM_H


#include <Graph.h>
#include <chrono>

typedef std::chrono::high_resolution_clock hrc;


class SCCAlgorithm {
public:
    SCCAlgorithm(Graph * graph_);

    const unordered_map<Node, Node> & getScc() const;

    virtual void execute() = 0;

    int getExecutionTime() const;


protected:
    Graph * graph;

    unordered_map<Node, Node> scc;

    int executionTime;

};



#endif //FEUP_CAL_PROJ_SCCALGORITHM_H
