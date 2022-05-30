#ifndef FEUP_CAL_PROJ_COSTFUNCTION_H
#define FEUP_CAL_PROJ_COSTFUNCTION_H

#include <TSPAlgorithm.h>
#include <Graph.h>

class CostFunction : public CostFunctionTSP {
public:
    CostFunction(unsigned n_, const unordered_map<Node, unordered_map<Node, Weight>> & cost_){
        n = n_;
        cost = cost_;
    }
    Weight operator()(const unordered_set<Node> & S, Node u, Node v) const override {
        return (n-S.size())*cost.at(u).at(v);
    }

private:
    unsigned n;

    unordered_map<Node, unordered_map<Node, Weight>> cost;

};


#endif //FEUP_CAL_PROJ_COSTFUNCTION_H
