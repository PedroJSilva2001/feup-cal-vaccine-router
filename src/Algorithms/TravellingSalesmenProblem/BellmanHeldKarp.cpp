#include "BellmanHeldKarp.h"

BellmanHeldKarp::BellmanHeldKarp(const unordered_set<Node> &V_, Node s_, CostFunctionTSP *cost_) : TSPAlgorithm(V_, s_, cost_) {
    nodeToIndex = unordered_multimap<Node, NodeIndex>();
    indexToNode = unordered_map<NodeIndex, Node>();
    NodeIndex index = 0;
    for(const Node & u: V){
        indexToNode.insert(make_pair(index, u));
        nodeToIndex.insert(make_pair(u, index++));
    }
    auto it = nodeToIndex.find(s_); if(it == nodeToIndex.end()) throw std::out_of_range("");
    sIndex = it->second;
    Bitmask fullBitmask = ALL_SET(V.size());
    P = vector<vector<pair<Bitmask, NodeIndex>>>(fullBitmask + 1,vector<pair<Bitmask, NodeIndex>>(V.size(), make_pair(0, INVALID_INDEX)));
    D = vector<vector<Weight>>(fullBitmask + 1,vector<Weight>(V.size(), INF));
}

void BellmanHeldKarp::execute() {
    auto start_time = hrc::now();
    Bitmask bitmask = ALL_SET(indexToNode.size());
    pair<Bitmask, NodeIndex> curr = make_pair(bitmask, INVALID_INDEX);
    Weight currCost = INF;
    for(NodeIndex u = 0; u < indexToNode.size(); ++u){
        if(u == sIndex) {
            continue;
        }
        Weight newCost = BHK_R(bitmask, u) + getCost(bitmask, u, sIndex);
        if(newCost < currCost){
            currCost = newCost;
            curr.second = u;
        }
    }
    route.insert(route.begin(), s);
    do {
        route.insert(route.begin(), indexToNode.at(curr.second));
        curr = P[curr.first][curr.second];
    } while (curr.second != sIndex);
    route.insert(route.begin(), s);
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

Weight BellmanHeldKarp::BHK_R(const Bitmask &bitmask, NodeIndex v){
    if(D[bitmask][v] != INF) {
        return D[bitmask][v];
    }
    if(!IS_SET(bitmask, sIndex)) {
        return INF;
    }
    if(v == sIndex){
        Bitmask emptyBitmask = NONE_SET;
        SET(emptyBitmask, sIndex);
        if (bitmask == emptyBitmask) {
            return (D[bitmask][v] = 0);
        }
        else {
            return INF;
        }
    }
    Bitmask bitmaskWithoutV = UNSET(bitmask, v);
    for(NodeIndex u = 0; u < indexToNode.size(); ++u){
        if(!IS_SET(bitmaskWithoutV, u)) continue;
        Weight newCost = BHK_R(bitmaskWithoutV, u) + getCost(bitmaskWithoutV, u, v);
        if(newCost < D[bitmask][v]){
            P[bitmask][v] = make_pair(bitmaskWithoutV, u);
            D[bitmask][v] = newCost;
        }
    }
    return D[bitmask][v];
}

Weight BellmanHeldKarp::getCost(const Bitmask & bitmask, NodeIndex u, NodeIndex v) const{
    unordered_set<Node> S = unordered_set<Node>();
    for(NodeIndex i = 0; i < indexToNode.size(); ++i) {
        if (IS_SET(bitmask, i)) {
            S.insert(indexToNode.at(i));
        }
    }
    return cost->operator()(S, indexToNode.at(u), indexToNode.at(v));
}
