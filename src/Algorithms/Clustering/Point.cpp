#include "Point.h"
#include <cmath>

Node Point::getNode() const {
    return node;
}

const pair<double, double> & Point::getCoords() const {
    return coords;
}

Node Point::getScc() const {
    return scc;
}

unsigned Point::getCluster() const {
    return cluster;
}

Weight Point::distance(const Point &p1, const Point &p2) {
    Weight dist = pow(p1.coords.first - p2.coords.first, 2) +
               pow(p1.coords.second - p2.coords.second, 2);
    return sqrt(dist);
}

Point::Point(pair<double, double> coords_, Node node_, Node scc_) {
    coords = coords_;
    node = node_;
    scc = scc_;
}

void Point::setCoords(const pair<double, double> & coords_) {
    coords = coords_;
}

bool Point::update(unsigned cluster_) {
    bool ret = cluster != cluster_;
    cluster = cluster_;
    return ret;
}
