#ifndef FEUP_CAL_PROJ_GRAPH_H
#define FEUP_CAL_PROJ_GRAPH_H


#include <unordered_map>
#include <unordered_set>
using namespace std;

typedef long long Node;
typedef long double Weight;
static const Node NULL_NODE = -1;
#define INF std::numeric_limits<Weight>::max()

class Edge {
public:
    Node dest;
    Weight weight;
    long long id;
    Weight capacity;
    Weight flow;

    Edge(Node dest_, Weight weight_, long long id_) {
        dest = dest_;
        weight = weight_;
        id = id_;
    }
    bool operator==(const Edge &rhs) const {
        return this->dest == rhs.dest;
    }
};

struct EdgeHash {
    int operator()(const Edge & e) const {
        return hash<long long>()(e.dest);
    }

    bool operator()(const Edge & l, const Edge & r) const {
        return l == r;
    }
};

class Graph {
public:
    unordered_set<Node> getNodeSet() const;
    void addNode(Node node);
    void removeNode(Node node);
    void addEdge(Node sourc, Node dest, Weight w, long long id);
    void removeEdge(Node sourc, Node dest);
    unordered_set<Edge, EdgeHash> & getOutgoingEdges(Node node);
    Graph getReversedGraph();

private:
    unordered_map< Node, unordered_set< Edge, EdgeHash > > adj;
    unordered_set< Node > nodeSet;
};


#endif //FEUP_CAL_PROJ_GRAPH_H
