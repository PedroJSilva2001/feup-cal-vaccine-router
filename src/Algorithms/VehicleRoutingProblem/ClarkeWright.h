#ifndef FEUP_CAL_PROJ_CLARKEWRIGHT_H
#define FEUP_CAL_PROJ_CLARKEWRIGHT_H

#include <VRPAlgorithm.h>
#include <Route.h>
class Saving {
public:
    Saving(unsigned i_, unsigned j_, Weight savings_);

    unsigned i;

    unsigned j;

    Weight value;
};

class ClarkeWright : public VRPAlgorithm {
public:
    ClarkeWright(const unordered_set<Node> & clients_, Node depot_, const unordered_map<Node, unordered_map<Node, Weight>> & cost_,
             const unordered_map<Node, unsigned int> & demand_, Weight t_, unsigned C_);

    void execute() override;

private:
    vector<vector<Node>> clusters;

    vector<Saving> savingsList;

    //vector<vector<Node>> routes_;

    unordered_map<Node, Route *> clientsToRoutes;
};


#endif //FEUP_CAL_PROJ_CLARKEWRIGHT_H
