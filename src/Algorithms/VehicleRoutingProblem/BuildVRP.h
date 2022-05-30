#ifndef FEUP_CAL_PROJ_BUILDVRP_H
#define FEUP_CAL_PROJ_BUILDVRP_H

#include <TSPAlgorithm.h>
#include <VRPAlgorithm.h>

class BuildVRP : public VRPAlgorithm {
public:
    BuildVRP(const vector<Node> & fullRoute_, const unordered_set<Node> & clients_, Node depot_, const unordered_map<Node, unordered_map<Node, Weight>> & cost_,
             const unordered_map<Node, unsigned int> & demand_, Weight t_, unsigned C_);

    void execute() override;

private:
    vector<Node> fullRoute;

};


#endif //FEUP_CAL_PROJ_BUILDVRP_H
