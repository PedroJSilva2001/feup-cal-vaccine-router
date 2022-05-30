#include "Kmeans.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <Point.h>
#include <map>
#include <FlowGraph.h>
using namespace std;

Kmeans::Kmeans(const unordered_set<Node> &depots_, const unordered_set<Node> &clients_,
               const unordered_map<Node, unordered_map<Node, Weight>> &cost_, unordered_map<Node, Node> &scc_,
               const unordered_map<Node, pair<double, double>> &coordinates_) : ClusteringAlgorithm(depots_, clients_, cost_, scc_) {
    coordinates = coordinates_;
    unassigned = unordered_set(clients);
    k = depots.size();
    points = vector<Point>();

    for (Node u : clients) {
        points.emplace_back(coordinates.at(u), u, scc.at(u));
    }

    vector<unsigned> pointIndices = vector<unsigned>();
    for (unsigned i = 0; i < points.size(); ++i) {
        pointIndices.push_back(i);
    }

    random_device rd;
    mt19937 rng(rd());
    shuffle(pointIndices.begin(), pointIndices.end(), rng);

    for (unsigned idx = 0; idx < k; idx++) {
        Point mean(points.at(pointIndices.at(idx)));
        means.push_back(mean);
    }
}

void Kmeans::execute() {
    auto start_time = hrc::now();

    executeNormal();

    unordered_map<unsigned, unordered_set<Node>> clusterIdxToNodes;
    for (unsigned idx = 0; idx < k; ++idx) {
        clusterIdxToNodes.insert(make_pair(idx, unordered_set<Node>()));
    }
    for (const Point & point : points) {
        clusterIdxToNodes.at(point.getCluster()).insert(point.getNode());
    }

    //Less otimized way of solving the matching problem

    /*unordered_set<Node> unassignedToCluster(depots);
    for (unsigned idx = 0; idx < k; idx++) {
        Weight currDist = INF;
        Node assignedDepot = NULL_NODE;
        for (Node depot : unassignedToCluster) {
            Weight newDist = getDist(depot, means.at(idx));
            if (currDist > newDist) {
                //cout << "cluster " << idx << "got depot " << depot << endl;
                currDist = newDist;
                assignedDepot = depot;
            }
        }
        unassignedToCluster.erase(assignedDepot);
        assignedClients.at(assignedDepot) = clusterIdxToNodes.at(idx);
    }*/

    //Making the flow graph for the matching problem (clients and clusters form a bipartite graph)
    //Add edge from all depots to all clusters whose weight is based on euclidian distance between them
    FlowGraph<Node> *graph = new FlowGraph<Node>();
    Node s = -1;
    Node t = -2;
    graph->addVertex(s);
    graph->addVertex(t);
    unordered_map<Node, unsigned> idToDepot = unordered_map<Node, unsigned>();
    unsigned id = k;
    for (Node u : depots) {
        idToDepot.insert(make_pair(id, u));
        graph->addVertex(id);
        graph->addEdge(s, id++, 1, 0);
    }
    for (unsigned idx = 0; idx < k; idx++) {
        graph->addVertex((Node)idx);
        graph->addEdge((Node)idx, t, 1, 0);
    }
    for (unsigned idDepot = k; idDepot < id; idDepot++) {
        for (unsigned idx = 0; idx < k; idx++) {
            graph->addEdge((Node)idDepot, (Node)idx, 1, getDist(idToDepot.at(idDepot), means.at(idx)));
        }
    }

    //Max flow minimum cost
    //We push k units of flow from source meaning at most we have k matchings (each cluster only gets one of the k depots
    // and vice-versa) because all capacities are 1, graph is bipartite and there is an edge from each client to all other clusters
    graph->minCostFlow(s, t, (double)k);
    for (Vertex<Node> *v : graph->getVertexSet()) {
        if (v->getInfo() < k) {
            continue;
        }
        for (EdgeF<Node> *e : v->getOutgoing()) {
            //Capacity in edge is 1 meaning that that edge corresponds to a matching in the problem
            if (e->getFlow() != 0) {
                assignedClients.at(idToDepot.at(v->getInfo())) = clusterIdxToNodes.at(e->getDest()->getInfo());
            }
        }
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

Weight Kmeans::getDist(Node u, const Point & p) {
    return sqrt(pow(coordinates.at(u).first - p.getCoords().first, 2) +
        pow(coordinates.at(u).second - p.getCoords().second, 2));
}

void Kmeans::executeNormal() {
    unsigned nIterations = 20;
    for (unsigned i = 0; i < nIterations; i++) {
        bool changed = assignToCluster();
        updateMeans();
        if (!changed) {
            return;
        }
    }
}

bool Kmeans::assignToCluster() {
    bool changed = false;
    for (Point & point : points) {
        unsigned minCluster = findNearestCluster(point);
        bool ret = point.update(minCluster);
        changed |= ret;
    }
    return changed;
}

void Kmeans::updateMeans() {
    multimap<unsigned, const Point *> clusterToPoint;
    for (const auto &point : points) {
        clusterToPoint.insert(make_pair(point.getCluster(), &point));
    }

    for (unsigned clusterIdx = 0; clusterIdx < k; ++clusterIdx) {
        computeClusterMean(clusterToPoint, clusterIdx, &means.at(clusterIdx));
    }
}

unsigned Kmeans::findNearestCluster(const Point & point) {
    Weight currDist = INF;
    unsigned minCluster = k + 1;
    for (unsigned idx = 0; idx < k; idx++) {
        Weight newDist = Point::distance(point, means.at(idx));
        if (currDist > newDist) {
            currDist = newDist;
            minCluster = idx;
        }
    }
    return minCluster;
}

void Kmeans::computeClusterMean(multimap<unsigned int, const Point *> multimap, unsigned cluster, Point *mean) {
    double x = 0.0;
    double y = 0.0;
    auto inCluster = multimap.equal_range(cluster);
    int numPoints = 0;
    for (auto itr = inCluster.first; itr != inCluster.second; ++itr) {
        x += (*itr).second->getCoords().first;
        y += (*itr).second->getCoords().second;
        numPoints++;
    }
    mean->setCoords(pair<double, double>(x/numPoints, y/numPoints));
}

