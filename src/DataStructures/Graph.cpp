#include "Graph.h"

unordered_set<Node> Graph::getNodeSet() const {
    return nodeSet;
}

void Graph::addNode(Node node) {
    if (!nodeSet.insert(node).second) {
        throw invalid_argument("Error adding node with id " + to_string(node) +" - node already exists in the graph");
    }
    unordered_set< Edge, EdgeHash> outGoingEdges;
    adj.insert(make_pair(node, outGoingEdges));

}

void Graph::removeNode(Node node) {
    if (nodeSet.erase(node) == 0) {
        throw invalid_argument("Error removing node with id " + to_string(node) +" - node doesn't exist in the graph");
    }
    adj.erase(node);
    for (auto & nodeEdges : adj) {
        nodeEdges.second.erase(Edge(node, -1, -1));
    }
}

void Graph::addEdge(Node sourc, Node dest, Weight w, long long id) {
    if (nodeSet.find(sourc) == nodeSet.end()) {
        throw std::invalid_argument("Error adding edge from node with id " + to_string(sourc) + " to node with id " + to_string(dest) + " - source node doesn't exist in the graph");
    }

    if (nodeSet.find(dest) == nodeSet.end()) {
        throw std::invalid_argument("Error adding edge from node with id " + to_string(sourc) + " to node with id " + to_string(dest) + " - target node doesn't exist in the graph");
    }
    Edge e(dest, w, id);
    if(!adj.at(sourc).insert(e).second) {
        throw std::invalid_argument("Error adding edge from node with id " + to_string(sourc) + " to node with id " + to_string(dest) + " edge already exists");
    }
}

void Graph::removeEdge(Node sourc, Node dest) {
    if (nodeSet.find(sourc) == nodeSet.end()) {
        throw std::invalid_argument("Error removing edge from node with id " + to_string(sourc) + " to node with id " + to_string(dest) + " - source node doesn't exist in the graph");
    }

    if (nodeSet.find(dest) == nodeSet.end()) {
        throw std::invalid_argument("Error removing edge from node with id " + to_string(sourc) + " to node with id " + to_string(dest) + " - target node doesn't exist in the graph");
    }

    Edge e(dest, -1, -1);
    if (adj.at(sourc).find(e) == adj.at(sourc).end()){
        throw std::invalid_argument("Error removing edge from node with id " + to_string(sourc) + " to node with id " + to_string(dest) + " - edge doesn't exist");
    }
    adj.at(sourc).erase(e);
}

unordered_set<Edge, EdgeHash> &Graph::getOutgoingEdges(Node node) {
    if (adj.find(node) == adj.end()) {
        throw invalid_argument("Error getting outgoing edges of node with id " + to_string(node) + " - node doesn't exist in the graph");
    }
    return adj.at(node);
}

Graph Graph::getReversedGraph() {
    Graph reversedGraph;
    for(Node u : nodeSet) {
        reversedGraph.addNode(u);
    }
    for (Node u : nodeSet) {
        for (Edge e : adj.at(u)) {
            reversedGraph.addEdge(e.dest, u, e.weight, e.id);
        }
    }
    return reversedGraph;
}