#include "Route.h"

Route::Route(const vector<Node> &route_, const unordered_map<Node, unordered_map<Node, Weight>> & cost) {
    route = route_;
    pair<Weight, Weight> weights = getRouteWeights(route, cost);
    fullRouteWeight = weights.first;
    routeWeightWithoutLast = weights.second;
    fullRouteWeight = 0;
    routeWeightWithoutLast = 0;
}

Route::Route(unsigned id_) {
    fullRouteWeight = 0;
    routeWeightWithoutLast = 0;
    route = vector<Node>();
    capacity = 0;
    id = id_;
}

pair<Weight, Weight> Route::getRouteWeights(const vector<Node> &route_, const unordered_map<Node, unordered_map<Node, Weight>> & cost) {
    Weight fullWeight = 0;
    Weight partialWeight = 0;
    for (unsigned i = 0; i < route_.size() - 1; i++) {
        Weight toAdd = cost.at(route_[i]).at(route_[i+1]);
        if (i != route_.size()-2) {
            partialWeight += toAdd;
        }
        fullWeight += toAdd;
    }
    return make_pair(fullWeight, partialWeight);
}

ostream & operator<<(ostream &os, const Route &r) {
    os << "Path: ";
    for (unsigned i = 0; i < r.getRoute().size(); i++) {
        cout << r.getRoute()[i];
        if (i != r.getRoute().size()-1) {
            cout << " -> ";
        }
    }
    cout << endl;
    os << "Full time of route: " << r.fullRouteWeight << endl;
    os << "Time of route without trip to depot: " << r.getRouteWeightWithoutLast() << endl;
    os << "Demand of route: " << r.getCapacity() << endl;
    return os;
}

Weight Route::getFullRouteWeight() const {
    return fullRouteWeight;
}

Weight Route::getRouteWeightWithoutLast() const {
    return routeWeightWithoutLast;
}

const vector<Node> &Route::getRoute() const {
    return route;
}

void Route::insert(Node u, const unordered_map<Node, unordered_map<Node, Weight>> & cost) {
    if (route.empty()) {
        route.push_back(u);
        return;
    }
    Weight costIncrement = cost.at(route.at(route.size()-1)).at(u);
    route.push_back(u);
    fullRouteWeight += costIncrement;
}

void Route::insert(Node u, Weight cost, unsigned demand) {
    route.push_back(u);
    routeWeightWithoutLast = fullRouteWeight;
    fullRouteWeight += cost;
    capacity += demand;
}

Node Route::getLastNode() {
    return route.at(route.size()-1);
}

unsigned Route::getCapacity() const {
    return capacity;
}

void Route::merge(Route *r, Weight newCost, Weight newCostWithoutLast, Weight newCapacity) {
    route.pop_back();
    for (unsigned i = 1; i < r->getRoute().size(); i++) {
        route.push_back(r->getRoute().at(i));
    }
    fullRouteWeight = newCost;
    routeWeightWithoutLast = newCostWithoutLast;
    capacity = newCapacity;
}

unsigned Route::getId() const {
    return id;
}

