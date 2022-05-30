#ifndef FEUP_CAL_PROJ_NAIVE_H
#define FEUP_CAL_PROJ_NAIVE_H


#include <ClusteringAlgorithm.h>

class Naive : public ClusteringAlgorithm {
public:
    Naive(const unordered_set<Node> & depots_, const unordered_set<Node> & clients_,
          const unordered_map<Node, unordered_map<Node, Weight>> & cost_, unordered_map<Node, Node> &scc_);

    void execute() override;

};


#endif //FEUP_CAL_PROJ_NAIVE_H
