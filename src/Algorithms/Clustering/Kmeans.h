#ifndef FEUP_CAL_PROJ_KMEANS_H
#define FEUP_CAL_PROJ_KMEANS_H

#include <ClusteringAlgorithm.h>
#include <map>
#include <msclus.h>
#include "Point.h"

using namespace std;
/*class Cluster {
public:
    void updateCentroid(const unordered_map<Node, pair<double, double>> & coordinates);

    void insert(Node u);

    void clear();

    const pair<double, double> & getCentroid() const;

    const vector<Node> & getNodes() const;

private:
    vector<Node> nodes;

    pair<double, double> centroid;

};*/


class Kmeans : public ClusteringAlgorithm {
public:
    Kmeans(const unordered_set<Node> & depots_, const unordered_set<Node> & clients_, const unordered_map<Node, unordered_map<Node, Weight>> & cost_,
           unordered_map<Node, Node> & scc_, const unordered_map<Node, pair<double, double>> & coordinates_);

    void execute() override;

    void executeNormal();

private:
    unordered_map<Node, pair<double, double>> coordinates;

    unordered_set<Node> unassigned;

    unsigned k;

    vector<Point> points;

    vector<Point> means;

    Weight getDist(Node u, const Point & p);

    bool assignToCluster();

    void updateMeans();

    unsigned findNearestCluster(const Point &point);

    void computeClusterMean(multimap<unsigned, const Point *> multimap, unsigned idx, Point * mean);
};


#endif //FEUP_CAL_PROJ_KMEANS_H
