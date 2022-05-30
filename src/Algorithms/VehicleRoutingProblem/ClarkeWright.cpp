#include "ClarkeWright.h"
#include <algorithm>

using namespace std;

ClarkeWright::ClarkeWright(const unordered_set<Node> & clients_, Node depot_,
                           const unordered_map<Node, unordered_map<Node, Weight>> & cost_,
                           const unordered_map<Node, unsigned int> & demand_, Weight t_,
                           unsigned int C_) : VRPAlgorithm(clients_, depot_, cost_, demand_, t_, C_) {
    clientsToRoutes = unordered_map<Node, Route*>();
}

void ClarkeWright::execute() {
    auto start_time = hrc::now();
    Weight savings;
    // Calculate savings list
    for (Node i : clients) {
        for (Node j : clients) {
            if (i == j) {
                continue;
            }
            savings = cost.at(i).at(depot) + cost.at(depot).at(j) - cost.at(i).at(j);
            savingsList.emplace_back(i, j, savings);
        }
    }
    // Sorting savings list in decreasing fashion
    sort(savingsList.begin(), savingsList.end(), [](const Saving & s1, const Saving & s2) {
        return s1.value > s2.value;
    });
    // Building routes
    unsigned id = 0;
    for (Node u : clients) {
        Route * r = new Route(id++);
        r->insert(depot, 0, 0);
        r->insert(u, cost.at(depot).at(u), demand.at(u));
        r->insert(depot, cost.at(u).at(depot), 0);
        clientsToRoutes.insert(make_pair(u, r));
    }
    // Main loop merging routes
    Route * routei;
    Route * routej;
    for (const Saving & savings : savingsList) {
        if (savings.value < 0) {
            break;
        }
        routei = clientsToRoutes.at(savings.i); // route starting in (0, i)
        routej = clientsToRoutes.at(savings.j);  // route ending in (j, 0)
        if (routei == routej) {
            continue;
        }
        // If route starts with i and other route ends in j
        if (routei->getRoute().at(1) == savings.i &&
        routej->getRoute().at(routej->getRoute().size()-2) == savings.j) {
            Weight toRemove = routej->getFullRouteWeight() - routej->getRouteWeightWithoutLast();
            Weight mergedCapacity = routei->getCapacity() + routej->getCapacity();
            Weight mergedCost = routei->getFullRouteWeight() + routej->getFullRouteWeight() - savings.value;
            if (mergedCapacity <= C && mergedCost - toRemove <= t) {
                routei->merge(routej, mergedCost, mergedCost - toRemove, mergedCapacity);
                // all nodes in a route starting in (0, i) get the route ending in (j, 0)
                for (Node u : routei->getRoute()) {
                    if (u == depot) {
                        continue;
                    }
                    clientsToRoutes.at(u) = routei;
                }
                delete routej;
           }
        }
    }
    unordered_set<unsigned> ids;
    for (Node u : clients) {
        // Don't know if this is needed
        if (clientsToRoutes.at(u) == nullptr) {
            continue;
        }
        // Without using ids the routes were being repeated in the vector :(
        if (ids.find(clientsToRoutes.at(u)->getId()) == ids.end()) {
            routes.push_back(clientsToRoutes.at(u));
            ids.insert(clientsToRoutes.at(u)->getId());
        }
    }
    executionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

Saving::Saving(unsigned int i_, unsigned int j_, Weight savings_) {
    i = i_;
    j = j_;
    value = savings_;
}
