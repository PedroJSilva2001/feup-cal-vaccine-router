#ifndef FEUP_CAL_PROJ_NEARESTNEIGHBOUR_H
#define FEUP_CAL_PROJ_NEARESTNEIGHBOUR_H

#include <TSPAlgorithm.h>

class NearestNeighbour : public TSPAlgorithm {
public:
    NearestNeighbour(const unordered_set<Node> & V_, Node s_, CostFunctionTSP *cost_);

    void execute() override;

private:
    unordered_set<Node> visited;
};


#endif //FEUP_CAL_PROJ_NEARESTNEIGHBOUR_H
