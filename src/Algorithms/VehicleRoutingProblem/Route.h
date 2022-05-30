#ifndef FEUP_CAL_PROJ_ROUTE_H
#define FEUP_CAL_PROJ_ROUTE_H

#include <Graph.h>
#include <vector>
#include <iostream>

class Route {
public:
    Route(const vector<Node> &route_, const unordered_map<Node, unordered_map<Node, Weight>> &cost);

    Route(unsigned id_);

    Weight getFullRouteWeight() const;

    Weight getRouteWeightWithoutLast() const;

    const vector<Node> & getRoute() const;

    friend ostream & operator<<(ostream &os, const Route &r);

    void insert(Node u, const unordered_map<Node, unordered_map<Node, Weight>> & cost);

    void insert(Node u, Weight cost, unsigned demand);

    Node getLastNode();

    unsigned getCapacity() const;

    void merge(Route * r, Weight newCost, Weight newCostWithoutLast, Weight newCapacity);

    unsigned getId() const;

private:
    Weight fullRouteWeight;

    Weight routeWeightWithoutLast;

    unsigned capacity;

    vector<Node> route;

    unsigned id;

    static pair<Weight, Weight> getRouteWeights(const vector<Node> & route_, const unordered_map<Node, unordered_map<Node, Weight>> & cost);

};


#endif //FEUP_CAL_PROJ_ROUTE_H
