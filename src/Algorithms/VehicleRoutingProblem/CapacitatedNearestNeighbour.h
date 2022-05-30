#ifndef FEUP_CAL_PROJ_CAPACITATEDNEARESTNEIGHBOUR_H
#define FEUP_CAL_PROJ_CAPACITATEDNEARESTNEIGHBOUR_H

#include <VRPAlgorithm.h>
#include <NodeWrapper.h>

class NodeWrapperNN {
public:
    Node node;
    Weight travelTime;
    NodeWrapperNN(Node node_, Weight travelTime_) : node(node_), travelTime(travelTime_) {};
    bool operator<(const NodeWrapperNN & rhs) const {
        return travelTime > rhs.travelTime;
    }
};

class CapacitatedNearestNeighbour : public VRPAlgorithm {
public:
    CapacitatedNearestNeighbour(const unordered_set<Node> & V_, Node s_, const unordered_map<Node, unordered_map<Node, Weight>> & cost_,
                                const unordered_map<Node, unsigned> & demand_, unsigned vehicleNumber_, Weight t_, unsigned C_);

    void execute() override;

private:
    unordered_set<Node> unvisited;

    unsigned vehicleNumber;
};


#endif //FEUP_CAL_PROJ_CAPACITATEDNEARESTNEIGHBOUR_H
