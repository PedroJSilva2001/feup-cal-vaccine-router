#ifndef FEUP_CAL_PROJ_POINT_H
#define FEUP_CAL_PROJ_POINT_H

using namespace std;
#include <Graph.h>
#include <vector>

class Point {
public:
    Point() {}

    Point(pair<double, double> coords_, Node node_, Node scc_);

    static Weight distance(const Point &p1, const Point &p2);

    bool update(unsigned cluster_);

    const pair<double, double> & getCoords() const;

    void setCoords(const pair<double, double> & coords_);

    unsigned getCluster() const;

    Node getScc() const;

    Node getNode() const;

private:
    pair<double, double> coords;

    unsigned cluster;

    Node scc;

    Node node;
};

#endif //FEUP_CAL_PROJ_POINT_H
