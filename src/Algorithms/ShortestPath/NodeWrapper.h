#ifndef FEUP_CAL_PROJ_NODEWRAPPER_H
#define FEUP_CAL_PROJ_NODEWRAPPER_H

#include <Graph.h>
class NodeWrapper {
public:
    Node node;
    long long queueIndex = -1;
    Weight dist = INF;

    NodeWrapper(Node node_, Weight dist_) {
        node = node_;
        dist = dist_;
    }

    bool operator<(const NodeWrapper &rhs) const {
        return this->dist < rhs.dist;
    }
};

#endif //FEUP_CAL_PROJ_NODEWRAPPER_H
