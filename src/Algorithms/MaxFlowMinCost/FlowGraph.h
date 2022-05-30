#ifndef MaxFlowMinCost_H_
#define MaxFlowMinCost_H_

#include <vector>
#include <queue>
#include <limits>
#include <iostream>
#include "MutablePriorityQueue.h"

using namespace std;
constexpr auto INFF = std::numeric_limits<double>::max();

template <class T> class Vertex;
template <class T> class EdgeF;
template <class T> class FlowGraph;

/*
 * ================================================================================================
 * Class Vertex
 * ================================================================================================
 */
template <class T>
class Vertex {
    T info;
    vector<EdgeF<T> *> outgoing;
    vector<EdgeF<T> *> incoming;

    bool visited;  // for path finding
    EdgeF<T> *path; // for path finding
    double dist;   // for path finding
    int queueIndex = 0; // required by MutablePriorityQueue

    Vertex(T in);
    void addEdge(EdgeF<T> *e);
    bool operator<(Vertex<T> & vertex) const; // required by MutablePriorityQueue

public:
    T getInfo() const;
    vector<EdgeF<T> *> getIncoming() const;
    vector<EdgeF<T> *> getOutgoing() const;
    friend class FlowGraph<T>;
    friend class MutablePriorityQueue<Vertex<T>>;
};

template <class T>
Vertex<T>::Vertex(T in): info(in) {}

template <class T>
void Vertex<T>::addEdge(EdgeF<T> *e) {
    outgoing.push_back(e);
    e->dest->incoming.push_back(e);
}

template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
vector<EdgeF<T> *>  Vertex<T>::getIncoming() const {
    return this->incoming;
}

template <class T>
vector<EdgeF<T> *>  Vertex<T>::getOutgoing() const {
    return this->outgoing;
}

/* ================================================================================================
 * Class Edge
 * ================================================================================================
 */
template <class T>
class EdgeF {
    Vertex<T> * orig;
    Vertex<T> * dest;
    double capacity;
    double cost;
    double flow;
    EdgeF(Vertex<T> *o, Vertex<T> *d, double capacity, double cost=0, double flow=0);

public:
    friend class FlowGraph<T>;
    friend class Vertex<T>;
    double getFlow() const;
    Vertex<T> * getDest() const;
    double getCost() const;
};

template <class T>
EdgeF<T>::EdgeF(Vertex<T> *o, Vertex<T> *d, double capacity, double cost, double flow):
        orig(o), dest(d), capacity(capacity), cost(cost), flow(flow){}

template <class T>
double EdgeF<T>::getFlow() const {
    return this->flow;
}

/* ================================================================================================
 * Class FlowGraph
 * ================================================================================================
 */
template <class T>
class FlowGraph {
    vector<Vertex<T> *> vertexSet;

    void dijkstraShortestPath(Vertex<T> *s);
    void bellmanFordShortestPath(Vertex<T> *s);
    bool relax(Vertex<T> *v, Vertex<T> *w, EdgeF<T> *e, double residual, double cost);

    void resetFlows();
    void reduceCosts();
    bool findAugmentationPath(Vertex<T> *s, Vertex<T> *t);
    void testAndVisit(queue< Vertex<T>*> &q, EdgeF<T> *e, Vertex<T> *w, double residual);
    double findMinResidualAlongPath(Vertex<T> *s, Vertex<T> *t);
    void augmentFlowAlongPath(Vertex<T> *s, Vertex<T> *t, double flow);

public:
    Vertex<T>* findVertex(const T &inf) const;
    vector<Vertex<T> *> getVertexSet() const;
    Vertex<T> *addVertex(const T &in);
    EdgeF<T> *addEdge(const T &sourc, const T &dest, double capacity, double cost, double flow=0);
    double getFlow(const T &sourc, const T &dest) const ;
    void fordFulkerson(T source, T target);
    double minCostFlow(T source, T sink, double flow);
};

template<class T>
Vertex<T> *EdgeF<T>::getDest() const {
    return dest;
}

template<class T>
double EdgeF<T>::getCost() const {
    return cost;
}

template <class T>
Vertex<T> * FlowGraph<T>::addVertex(const T &in) {
    Vertex<T> *v = findVertex(in);
    if (v != nullptr)
        return v;
    v = new Vertex<T>(in);
    vertexSet.push_back(v);
    return v;
}

template <class T>
EdgeF<T> * FlowGraph<T>::addEdge(const T &sourc, const T &dest, double capacity, double cost, double flow) {
    auto s = findVertex(sourc);
    auto d = findVertex(dest);
    if (s == nullptr || d == nullptr)
        return nullptr;
    EdgeF<T> *e = new EdgeF<T>(s, d, capacity, cost, flow);
    s->addEdge(e);
    return e;
}

template <class T>
Vertex<T>* FlowGraph<T>::findVertex(const T & inf) const {
    for (auto v : vertexSet)
        if (v->info == inf)
            return v;
    return nullptr;
}

template <class T>
double FlowGraph<T>::getFlow(const T &sourc, const T &dest) const {
    auto s = findVertex(sourc);
    auto d = findVertex(dest);
    if (s == nullptr || d == nullptr)
        return 0.0;
    for (auto e : s->outgoing)
        if (e->dest == d)
            return e->flow;
    return 0.0;
}

template <class T>
vector<Vertex<T> *> FlowGraph<T>::getVertexSet() const {
    return vertexSet;
}

/**************** Maximum Flow Problem  ************/
/**
 * Finds the maximum flow in a FlowGraph using the Ford Fulkerson algorithm
 * (with the improvement of Edmonds-Karp).
 * Assumes that the FlowGraph forms a flow network from source vertex 's'
 * to sink vertex 't' (distinct vertices).
 * Receives as arguments the source and target vertices (identified by their contents).
 * The result is defined by the "flow" field of each edge.
 */
template <class T>
void FlowGraph<T>::fordFulkerson(T source, T target) {
    // Obtain the source (s) and target (t) vertices
    Vertex<T>* s = findVertex(source);
    Vertex<T>* t = findVertex(target);

    if (s == nullptr || t == nullptr || s == t)
        throw "Invalid source and/or target vertex";

    // Apply algorithm as in slides
    resetFlows();
    while( findAugmentationPath(s, t) ) {
        double f = findMinResidualAlongPath(s, t);
        augmentFlowAlongPath(s, t, f);
    }
}

template <class T>
void FlowGraph<T>::resetFlows() {
    for (auto v : vertexSet)
        for (auto e: v->outgoing)
            e->flow = 0;
}

template<class T>
bool FlowGraph<T>::findAugmentationPath(Vertex<T> *s, Vertex<T> *t) {
    for(auto v : vertexSet)
        v->visited = false;

    s->visited = true;
    queue< Vertex<T>* > q;
    q.push(s);

    while( ! q.empty() && ! t->visited) {
        auto v = q.front();
        q.pop();
        for(auto e: v->outgoing)
            testAndVisit(q, e, e->dest, e->capacity - e->flow);
        for(auto e: v->incoming)
            testAndVisit(q, e, e->orig, e->flow);
    }
    return t->visited;
}

/**
 * Auxiliary function used by findAugmentationPath.
 */
template<class T>
void FlowGraph<T>::testAndVisit(queue< Vertex<T>*> &q, EdgeF<T> *e, Vertex<T> *w, double residual) {
    if (!w-> visited && residual > 0 && e->cost == 0) {
        w->visited = true;
        w->path = e;
        q.push(w);
    }
}

template<class T>
double FlowGraph<T>::findMinResidualAlongPath(Vertex<T> *s, Vertex<T> *t) {
    double f = INFF;
    for (auto v = t; v != s; ) {
        auto e = v->path;
        if (e->dest == v) {
            f = min(f, e->capacity - e->flow);
            v = e->orig;
        }
        else {
            f = min(f, e->flow);
            v = e->dest;
        }
    }
    return f;
}

template<class T>
void FlowGraph<T>::augmentFlowAlongPath(Vertex<T> *s, Vertex<T> *t, double f) {
    for (auto v = t; v != s; ) {
        auto e = v->path;
        if (e->dest == v) {
            e->flow += f;
            v = e->orig;
        }
        else {
            e->flow -= f;
            v = e->dest;
        }
    }
}

/**************** Minimum Cost Flow Problem  ************/
/**
 * Determines the minimum cost flow in a flow network.
 * Receives as arguments the source and sink vertices (identified by their info),
 * and the intended flow.
 * Returns the calculated minimum cost for delivering the intended flow (or the highest
 * possible flow, if the intended flow is higher than supported by the network).
 * The calculated flow in each edge can be consulted with the "getFlow" function.
 * Notice: Currently, the costs of the edges are modified by the algorithm.
 */
template <class T>
double FlowGraph<T>::minCostFlow(T source, T sink, double flow) {
    // Obtain the source (s) and sink (t) vertices
    Vertex<T>* s = findVertex(source);
    Vertex<T>* t = findVertex(sink);

    if (s == nullptr || t == nullptr || s == t)
        throw "Invalid source and/or target vertex";

    resetFlows();
    double cur_cost = 0.0;
    double cur_flow = 0.0;
    double unit_cost = 0.0;

    bool firstIteration=true;
    while (cur_flow < flow) {
        if (firstIteration) {
            bellmanFordShortestPath(s);
            firstIteration=false;
        }
        else
            dijkstraShortestPath(s);
        if (t->dist == INFF)
            break;
        unit_cost += t->dist;
        reduceCosts();

        while (cur_flow < flow && findAugmentationPath(s, t)) {
            double f = min(findMinResidualAlongPath(s, t), flow - cur_flow);
            augmentFlowAlongPath(s, t, f);
            cur_flow += f;
            cur_cost += f * unit_cost;
        }
    }
    return cur_cost;
}

/**
 * Computes the shortest distance (with minimum cost) from "s" to all other vertices
 * in the residuals FlowGraph, using only edges with non-null residuals,
 * based on the Dijkstra algorithm.
 * The result is indicated by the field "dist" of each vertex.
 */
template<class T>
void FlowGraph<T>::dijkstraShortestPath(Vertex<T> *s ) {
    for(auto v : vertexSet)
        v->dist = INFF;
    s->dist = 0;
    MutablePriorityQueue<Vertex<T>> q;
    q.insert(s);
    while( ! q.empty() ) {
        auto v = q.extractMin();
        for (auto e : v->outgoing) {
            auto oldDist = e->dest->dist;
            if (relax(v, e->dest, e, e->capacity - e->flow, e->cost)){
                if (oldDist==INFF)
                    q.insert(e->dest);
                else
                    q.decreaseKey(e->dest);
            }
        }
        for (auto e : v->incoming) {
            auto oldDist = e->orig->dist;
            if (relax(v, e->orig, e, e->flow, -e->cost)) {
                if (oldDist == INFF)
                    q.insert(e->orig);
                else
                    q.decreaseKey(e->orig);
            }
        }
    }
}

/**
 * Computes the shortest distance (with minimum cost) from "s" to all other vertices
 * in the residuals FlowGraph, using only edges with non-null residuals,
 * based on the Bellman-Ford algorithm.
 * The result is indicated by the field "dist" of each vertex.
 */
template<class T>
void FlowGraph<T>::bellmanFordShortestPath(Vertex<T> *s ) {
    for(auto v : vertexSet)
        v->dist = INFF;

    s->dist = 0;
    for (unsigned i = 1; i < vertexSet.size(); i++)
        for (auto v: vertexSet) {
            for (auto e : v->outgoing)
                relax(v, e->dest, e, e->capacity - e->flow, e->cost);
            for (auto e : v->incoming)
                relax(v, e->orig, e, e->flow, -e->cost);
        }
}

/**
 * Auxiliary function used by Dijkstra and Bellman-Ford algorithms.
 * Analyzes edge (v, w) with a given residual and cost.
 */
template<class T>
bool FlowGraph<T>::relax(Vertex<T> *v, Vertex<T> *w, EdgeF<T> *e, double residual, double cost) {
    if (residual > 0 && v->dist + cost < w->dist) {
        w->dist = v->dist + cost;
        w->path = e;
        return true;
    }
    else
        return false;
}

/**
 * Reduces edges' costs, based on the vertices potentials
 * (shortest distances to 's', when not infinite).
 */
template <class T>
void FlowGraph<T>::reduceCosts() {
    for (auto v : vertexSet)
        for (auto e: v->outgoing)
            if (e->orig->dist != INFF && e->dest->dist != INFF)
                e->cost += e->orig->dist - e->dest->dist;
}

#endif /* FlowGraph_H_ */