#ifndef FEUP_CAL_PROJ_BELLMANHELDKARP_H
#define FEUP_CAL_PROJ_BELLMANHELDKARP_H

#include <TSPAlgorithm.h>

using namespace std;

typedef unsigned short NodeIndex;
typedef unsigned long long Bitmask;
static const NodeIndex INVALID_INDEX = std::numeric_limits<NodeIndex>::max();

#define BIT(n) (1uL << n)
#define UNSET(A, i) (A & (~BIT(i)))
#define IS_SET(A, i) (A & BIT(i))
#define ALL_SET(n) (BIT(n)-1)
#define NONE_SET 0uL
#define SET(A, i) (A |= BIT(i))


class BellmanHeldKarp : public TSPAlgorithm {
public:
    BellmanHeldKarp(const unordered_set<Node> &V_, Node s_, CostFunctionTSP *cost_);

    void execute() override;

private:
    NodeIndex sIndex = INVALID_INDEX;

    unordered_multimap<Node, NodeIndex> nodeToIndex;

    unordered_map<NodeIndex, Node> indexToNode;

    vector<vector<Weight>> D;

    vector<vector<pair<Bitmask, NodeIndex>>> P;

    Weight BHK_R(const Bitmask & bitmask, NodeIndex v);

    Weight getCost(const Bitmask & bitmask, NodeIndex u, NodeIndex v) const;

};


#endif //FEUP_CAL_PROJ_BELLMANHELDKARP_H
