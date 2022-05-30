#ifndef FEUP_CAL_PROJ_TIMEASTARHEURISTIC_H
#define FEUP_CAL_PROJ_TIMEASTARHEURISTIC_H

#include <Astar.h>
#include <cmath>

class TimeAstarHeuristic : public AstarHeuristic {
public:
    TimeAstarHeuristic(Node t_, const unordered_map<Node, pair<double, double>> & coordinates_, unsigned fastestVelocity_);

    Weight operator()(Node u) const override;

private:
    Node t;

    unordered_map<Node, pair<double, double>> coordinates;

    unsigned fastestVelocity;

};

TimeAstarHeuristic::TimeAstarHeuristic(Node t_, const unordered_map<Node, pair<double, double>> &coordinates_,
                                       unsigned int fastestVelocity_) {
    t = t_;
    coordinates = coordinates_;
    fastestVelocity = fastestVelocity_;
}

Weight TimeAstarHeuristic::operator()(Node u) const {
    Weight dist = sqrt(pow(coordinates.at(u).first - coordinates.at(t).first, 2) +
                                    pow(coordinates.at(u).second - coordinates.at(t).second, 2));
    return dist/1000/(double)fastestVelocity;
}

#endif //FEUP_CAL_PROJ_TIMEASTARHEURISTIC_H
