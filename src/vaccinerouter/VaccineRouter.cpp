#include "VaccineRouter.h"
#include <Dijkstra.h>
#include <Tarjan.h>
#include <MutablePriorityQueue.h>
#include <Naive.h>
#include <Kmeans.h>
#include <NearestNeighbour.h>
#include <CostFunction.h>
#include <BuildVRP.h>
#include <BellmanHeldKarp.h>
#include <iostream>
#include <cstdlib>
#include <DepthFirstSearch.h>
#include <ClarkeWright.h>

using namespace std::chrono;

VaccineRouter::VaccineRouter(Graph * graph_, const unordered_set<Node> & depots_, const unordered_set<Node> & clients_,
                             const unordered_map<Node, unsigned int> & demand_, const unordered_map<Node, pair<double, double>> & coordinates_,
                             Constraints *constraints_) {
    graph = graph_;
    constraints = constraints_;
    randInfo = nullptr;
    depots = depots_;
    clients = clients_;
    demand = demand_;
    cost = unordered_map<Node, unordered_map<Node, Weight>>();
    prev = unordered_map<Node, unordered_map<Node, Node>>();
    assignedClients = unordered_map<Node, unordered_set<Node>>();
    coordinates = coordinates_;
    nodeNumberInEachScc = unordered_map<Node, long long>();
    routes = unordered_map<Node, vector<Route*>>();
    reachable = unordered_map<Node, vector<Node>>();
    computeScc();
    computeShortestPaths();
    setNodesInBiggestScc();
    //voidComputeReachabilities();
    for (Node depot : depots) {
        routes.insert(make_pair(depot, vector<Route*>()));
    }
}

VaccineRouter::VaccineRouter(Graph *graph_, const unordered_map<Node, pair<double, double>> &coordinates_, Constraints *constraints_, RandInfo *randInfo_) {
    graph = graph_;
    constraints = constraints_;
    randInfo = randInfo_;
    depots = unordered_set<Node>();
    clients = unordered_set<Node>();
    cost = unordered_map<Node, unordered_map<Node, Weight>>();
    prev = unordered_map<Node, unordered_map<Node, Node>>();
    assignedClients = unordered_map<Node, unordered_set<Node>>();
    coordinates = coordinates_;
    routes = unordered_map<Node, vector<Route*>>();
    reachable = unordered_map<Node, vector<Node>>();
    computeScc();
    setNodesInBiggestScc();
    setRandomClients();
    setRandomDepots();
    computeShortestPaths();
    voidComputeReachabilities();
    for (Node depot : depots) {
        routes.insert(make_pair(depot, vector<Route*>()));
    }
    setRandomDemands();
}

void VaccineRouter::computeScc() {
    SCCAlgorithm * sccAlgorithm = new Tarjan(graph);
    cout << "Computing strongly connected components of the graph..\n";
    sccAlgorithm->execute();
    cout << "SCCs computed in " << sccAlgorithm->getExecutionTime() << " milliseconds"<<endl << endl;
    scc = sccAlgorithm->getScc();
}

void VaccineRouter::computeShortestPaths() {
    MutablePriorityQueue<NodeWrapper> pq;
    Dijkstra * dijkstra;
    int shortestPathExecutionTime = 0;
    cout << "Computing all shortest paths with points interest as source..\n";
    for (Node u : depots) {
        dijkstra = new Dijkstra(pq, graph, u, NULL_NODE);
        dijkstra->execute();
        shortestPathExecutionTime += dijkstra->getExecutionTime();
        cost.insert(make_pair(u, dijkstra->getDist()));
        prev.insert(make_pair(u, dijkstra->getPrev()));
    }
    for (Node u : clients) {
        dijkstra = new Dijkstra(pq, graph, u, NULL_NODE);
        dijkstra->execute();
        shortestPathExecutionTime += dijkstra->getExecutionTime();
        cost.insert(make_pair(u, dijkstra->getDist()));
        prev.insert(make_pair(u, dijkstra->getPrev()));
        assignedClients.insert(make_pair(u, unordered_set<Node>()));
    }
    cout << "Shortest paths computed in " << shortestPathExecutionTime << " milliseconds"<<endl << endl;
}

void VaccineRouter::printSccCount() {
    for (Node u : depots) {
        std::cout << "Depot " << u << endl;
        for (Node v : assignedClients.at(u)) {
            std::cout << v << " " << cost.at(u).at(v) << endl;
        }
        cout << endl;
    }
}

void VaccineRouter::assignClientsToDepots(ClusteringAlgorithm *clusteringAlgorithm) {
    clusteringAlgorithm->execute();
    assignedClients = clusteringAlgorithm->getAssignedClients();
    assignementExecutionTime = clusteringAlgorithm->getExecutionTime();
}

void VaccineRouter::assignNaive() {
    ClusteringAlgorithm * clusteringAlgorithm = new Naive(depots, clients, cost, scc);
    assignClientsToDepots(clusteringAlgorithm);
}

void VaccineRouter::assignKmeans() {
    ClusteringAlgorithm * clusteringAlgorithm = new Kmeans(depots, clients, cost, scc, coordinates);
    assignClientsToDepots(clusteringAlgorithm);
}

void VaccineRouter::addRoutes(VRPAlgorithm * vrpAlgorithm, Node depot) {
    vrpAlgorithm->execute();
    routes.at(depot) = vrpAlgorithm->getRoutes();
}

void VaccineRouter::solveClarkWright() {
    VRPAlgorithm * vrpAlgorithm;
    routingExecutionTime = 0;
    auto start_time = hrc::now();
    for (Node depot : depots) {
        vrpAlgorithm = new ClarkeWright(assignedClients.at(depot), depot, cost, demand, constraints->t, constraints->C);
        //addRoutes(vrpAlgorithm, depot);
        vrpAlgorithm->execute();
        routes.at(depot) = vrpAlgorithm->getRoutes();
        //routingExecutionTime += vrpAlgorithm->getExecutionTime();
    }
    routingExecutionTime += std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

void VaccineRouter::solveNearestNeighbour() {
    TSPAlgorithm * tspAlgorithm;
    VRPAlgorithm * vrpAlgorithm;
    //routingExecutionTime = 0;
    auto start_time = hrc::now();
    for (Node depot : depots) {
        unordered_set<Node> V(assignedClients.at(depot).begin(), assignedClients.at(depot).end());
        V.insert(depot);
        CostFunctionTSP * costFunctionTsp = new CostFunction(V.size(), cost);
        tspAlgorithm = new NearestNeighbour(V, depot, costFunctionTsp);
        tspAlgorithm->execute();
        vector<Node> fullRoute = tspAlgorithm->getRoute();
        /*cout << "depot " << depot << endl;
        for (Node u : fullRoute) {
            cout << u << " ";
        }
        cout << "\n____________________________" << endl;
        cout << endl;
        cout << endl;*/
        vrpAlgorithm = new BuildVRP(fullRoute, assignedClients.at(depot), depot, cost, demand, constraints->t, constraints->C);
        vrpAlgorithm->execute();
        routes.at(depot) = vrpAlgorithm->getRoutes();
        //routingExecutionTime += (tspAlgorithm->getExecutionTime() + vrpAlgorithm->getExecutionTime());
        //cout << endl;
        /*for (Route * r : vrpAlgorithm->getRoutes()) {
            for (Node u : r->getRoute()) {
                cout << u <<
            }
        }*/
    }
    routingExecutionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();

}

void VaccineRouter::solveBellmanHeldKarp() {
    TSPAlgorithm * tspAlgorithm;
    VRPAlgorithm * vrpAlgorithm;
    const unsigned BHK_MAX_N = 12;
    //cout << "held karp" << endl;
    routingExecutionTime = 0;
    auto start_time = hrc::now();
    for (Node depot : depots) {
        vector<Node> VT(assignedClients.at(depot).begin(), assignedClients.at(depot).end());
        /*cout << "depot " << depot<< endl;
        for (Node u : VT) {
            cout << u << endl;
        }
        cout << endl;*/
        unordered_set<Node> V = unordered_set<Node>();
        vector<unordered_set<Node>> Vs = vector<unordered_set<Node>>();
        unsigned n = 0;
        for (unsigned i = 0; i < VT.size(); i++) {
            n++;
            //cout << n << endl;
            if (n == BHK_MAX_N) {
                V.insert(depot);
                Vs.push_back(V);
                n = 0;
                V.clear();
            }
            V.insert(VT.at(i));
            if (i == VT.size()-1) {
                V.insert(depot);
                Vs.push_back(V);
            }
        }
        for (unordered_set<Node> & V_ : Vs) {
            CostFunctionTSP * costFunctionTsp = new CostFunction(V_.size(), cost);
            tspAlgorithm = new BellmanHeldKarp(V_, depot, costFunctionTsp);
            tspAlgorithm->execute();
            vector<Node> fullRoute = tspAlgorithm->getRoute();
            V_.erase(depot);
            vrpAlgorithm = new BuildVRP(fullRoute, V_, depot, cost, demand, constraints->t, constraints->C);
            vrpAlgorithm->execute();
            routingExecutionTime += tspAlgorithm->getExecutionTime() + vrpAlgorithm->getExecutionTime();
            for (Route * r : vrpAlgorithm->getRoutes()) {
                routes.at(depot).push_back(r);
            }
        }
    }
    routingExecutionTime = std::chrono::duration_cast<std::chrono::milliseconds>(hrc::now()-start_time).count();
}

const unordered_map<Node, vector<Route*>> &VaccineRouter::getRoutes() const {
    return routes;
}

void VaccineRouter::setBiggestSccRepNode() {
    setNodeNumberInEachScc();
    Node biggestScc;
    long cnt = 0;
    for (auto pair : nodeNumberInEachScc) {
        if (cnt < pair.second) {
            biggestScc = pair.first;
            cnt = pair.second;
        }
    }
    biggestSccRepNode = biggestScc;
}

void VaccineRouter::setNodeNumberInEachScc() {
    unordered_map<Node, long long> sccCount = unordered_map<Node, long long>();
    for(Node u : graph->getNodeSet()) {
        if (sccCount.find(scc.at(u)) == sccCount.end()) {
            sccCount.insert(make_pair(scc.at(u),1));
        }
        else {
            sccCount.at(scc.at(u))++;
        }
    }
    nodeNumberInEachScc = sccCount;
}

void VaccineRouter::setRandomDepots() {
    while (depots.size() != randInfo->depotNum) {  //
        unsigned long long i = rand() % nodesInBiggestScc.size();
        if (clients.find(nodesInBiggestScc[i]) == clients.end()) {
            depots.insert(nodesInBiggestScc[i]);
        }
    }
}

void VaccineRouter::setRandomClients() {
    while (clients.size() != randInfo->clientNum) { // 60
        unsigned long long i = rand() % nodesInBiggestScc.size();
        clients.insert(nodesInBiggestScc[i]);
    }
}

void VaccineRouter::setNodesInBiggestScc() {
    setBiggestSccRepNode();
    nodesInBiggestScc = vector<Node>();
    for (Node u : graph->getNodeSet()) {
        if (scc.at(u) == biggestSccRepNode) {
            nodesInBiggestScc.push_back(u);
        }
    }
}

const unordered_set<Node> & VaccineRouter::getDepots() const {
    return depots;
}

const unordered_set<Node> & VaccineRouter::getClients() const {
    return clients;
}

void VaccineRouter::setRandomDemands() {
    for (Node client : clients) {// 49 + 100
        unsigned dem = (rand() % randInfo->demandVariable) + randInfo->demandLowerLimit;
        demand.insert(make_pair(client, dem));
    }
}

void VaccineRouter::mapDrawing(GraphViewer &gv){
    GraphViewer::Color color[7] = {GraphViewer::RED, GraphViewer::BLUE, GraphViewer::ORANGE,
                                   GraphViewer::MAGENTA,GraphViewer::YELLOW,GraphViewer::CYAN,
                                   GraphViewer::PINK};
    int i = 0;
    for(Node depot : depots){
        GraphViewer::Node &n = gv.getNode(depot);
        n.setOutlineThickness(1);
        n.setSize(50);
        n.setColor(color[i]);
        i++;
    }
    for(Node client : clients){
        GraphViewer::Node &n = gv.getNode(client);
        n.setOutlineThickness(2);
        n.setSize(35);
        n.setColor(GraphViewer::GREEN);
    }
    i = 0;
}

void VaccineRouter::voidComputeReachabilities() {
    DepthFirstSearch * dfs;
    unordered_set<Node> poi = unordered_set<Node>(clients);
    int reachabilitiesExecutionTime = 0;
    for (Node u : depots) {
        poi.insert(u);
    }
    cout << "Computing all reachable nodes with points interest as source..\n";
    for (Node u : poi) {
        dfs = new DepthFirstSearch(graph, u, poi);
        dfs->execute();
        reachabilitiesExecutionTime += dfs->getExecutionTime();
        reachable.insert(make_pair(u, dfs->getReachable()));
    }
    cout << "Shortest paths computed in " << reachabilitiesExecutionTime << " milliseconds"<<endl << endl;
}

int VaccineRouter::pathDrawing(GraphViewer &gv){
    GraphViewer::Color color[7] = {GraphViewer::RED, GraphViewer::BLUE, GraphViewer::ORANGE,
                                   GraphViewer::MAGENTA,GraphViewer::YELLOW,GraphViewer::CYAN,
                                   GraphViewer::PINK};
    int i = 0;
    int j = 500000;
    for(Node depot : depots){
        vector<Route*> routeVec = routes.at(depot);

        cout << "Depot: " << depot << ", num of routes: " << routeVec.size() << endl;
//            printf("Depot: %lld, num of routes: %llu\n",depot, routeVec.size());
        for(Route *route : routeVec){
            vector<Node> nodeVec = route->getRoute();
            cout << "\tRoute with num of nodes: " << nodeVec.size() << endl;
//            printf("\tRoute with num of nodes: %llu\n", nodeVec.size());

            for(unsigned  a = nodeVec.size()-1; a > 0; a--){
                Node previews = nodeVec[a-1];
                Node next = nodeVec[a];
                Node node2 = next;
                Node node1 = next;
                while(node2 != previews) {
                    node2 = prev.at(previews).at(node2);
                    GraphViewer::Edge &e1 = gv.addEdge(j++, gv.getNode(node2), gv.getNode(node1),GraphViewer::Edge::EdgeType::UNDIRECTED);
                    e1.setColor(color[i]);
                    e1.setThickness(0.1);
                    node1 = node2;
                }
            }
        }
        i++;
        i%=7;
    }
    return j;
}

int VaccineRouter::pathDrawingAt(Node depot, GraphViewer &gv){
    GraphViewer::Color color[7] = {GraphViewer::RED, GraphViewer::BLUE, GraphViewer::ORANGE,
                                   GraphViewer::MAGENTA,GraphViewer::YELLOW,GraphViewer::CYAN,
                                   GraphViewer::PINK};
    int j = 500000;
    int i = 0;
    vector<Route*> routeVec = routes.at(depot);
    printf("Depot: %lld, num of routes: %llu\n",depot, routeVec.size());
    for(Route *route : routeVec){
        vector<Node> nodeVec = route->getRoute();
        printf("\tRoute with num of nodes: %llu\n", nodeVec.size());
        for(unsigned  a = nodeVec.size()-1; a > 0; a--){
            Node previews = nodeVec[a-1];
            Node next = nodeVec[a];
            Node node2 = next;
            Node node1 = next;
            while(node2 != previews) {
                node2 = prev.at(previews).at(node2);
                GraphViewer::Edge &e1 = gv.addEdge(j++, gv.getNode(node2), gv.getNode(node1),GraphViewer::Edge::EdgeType::UNDIRECTED);
                e1.setColor(color[i]);
                e1.setThickness(0.1);
                node1 = node2;
            }
        }
        i++;
        i%=7;
    }
    return j;
}

void VaccineRouter::animate(int lastEdgeId, GraphViewer &gv){
    gv.lock();
    for(int i = lastEdgeId-1;i>=500000;i--){
        if(i%10==0){
            gv.unlock();
            Sleep(10);
            gv.lock();
        }
        GraphViewer::Edge &e = gv.getEdge(i);
        e.setThickness(8.0);
    }
    gv.unlock();
}

void VaccineRouter::animateInst(int lastEdgeId, GraphViewer &gv){
    gv.lock();
    for(int i = lastEdgeId-1;i>=500000;i--){
        GraphViewer::Edge &e = gv.getEdge(i);
        e.setThickness(8.0);
    }
    gv.unlock();
}

const unordered_map<Node, long long> &VaccineRouter::getNodeNumberInEachScc() {
    return nodeNumberInEachScc;
}

const unordered_map<Node, vector<Node>> &VaccineRouter::getReachabilities() const {
    return reachable;
}

void VaccineRouter::drawMostPopulatedScc(GraphViewer &gv) {
    GraphViewer::Color color[8] = {GraphViewer::RED, GraphViewer::BLUE, GraphViewer::ORANGE,
                                   GraphViewer::GREEN, GraphViewer::MAGENTA,GraphViewer::YELLOW,
                                   GraphViewer::CYAN, GraphViewer::PINK};
    vector<pair<Node, long long>> v;
    unordered_map<Node, unsigned> repToIndex;
    for (pair<Node, long long> pair : nodeNumberInEachScc) {
        v.push_back(pair);
    }
    sort(v.begin(), v.end(), [](const pair<Node, long long> & p1,const pair<Node, long long> & p2 ) {
        return p1.second > p2.second;
    });

    for (unsigned i = 0; i < 8; i++) {
        repToIndex.insert(make_pair(v[i].first, i));
    }

    for (auto pair : scc) { //no -> rep
        if (repToIndex.find(pair.second) != repToIndex.end()) {
            int i = repToIndex.at(pair.second);
            GraphViewer::Node &n = gv.getNode(pair.first);
            n.setSize(10);
            n.setColor(color[i]);
        }
    }
}

void VaccineRouter::hideNodes(GraphViewer &gv) {
    for (auto pair : scc) { //no -> rep
        GraphViewer::Node &n = gv.getNode(pair.first);
        n.setSize(0.0);
        n.setColor(GraphViewer::BLACK);
    }
}

void VaccineRouter::drawAssignedClients(GraphViewer &gv){
    GraphViewer::Color color[7] = {GraphViewer::RED, GraphViewer::BLUE, GraphViewer::ORANGE,
                                   GraphViewer::MAGENTA,GraphViewer::YELLOW,GraphViewer::CYAN,
                                   GraphViewer::PINK};
    int i = 0;
    for(Node depot : depots){
        GraphViewer::Node &n = gv.getNode(depot);
        n.setOutlineThickness(1);
        n.setSize(60);
        n.setColor(color[i]);
        for(Node client : assignedClients.at(depot)) {
            GraphViewer::Node &n = gv.getNode(client);
            n.setOutlineThickness(2);
            n.setSize(25);
            n.setColor(color[i]);
        }
        i++;
        i%=7;
    }
}

void VaccineRouter::drawAssignedClientsAt(Node depot, GraphViewer &gv){
    GraphViewer::Color color[7] = {GraphViewer::RED, GraphViewer::BLUE, GraphViewer::ORANGE,
                                   GraphViewer::MAGENTA, GraphViewer::YELLOW, GraphViewer::CYAN,
                                   GraphViewer::PINK};
    GraphViewer::Node &n = gv.getNode(depot);
    n.setOutlineThickness(1);
    n.setSize(60);
    n.setColor(color[0]);
    for(Node client : assignedClients.at(depot)) {
        GraphViewer::Node &n = gv.getNode(client);
        n.setOutlineThickness(2);
        n.setSize(25);
        n.setColor(color[0]);
    }
}

int VaccineRouter::getAssigmentExecutionTime() const {
    return assignementExecutionTime;
}

int VaccineRouter::getRoutingExecutionTime() const {
    return routingExecutionTime;
}

const unordered_map<Node, unordered_set<Node>> &VaccineRouter::getAssignedClients() const {
    return assignedClients;
}

const unordered_map<Node, unordered_map<Node, Weight>> &VaccineRouter::getCost() const {
    return cost;
}

int VaccineRouter::drawTSP(TSPAlgorithm *tsp, vector<Node> route, GraphViewer &gv){
    gv.setScale(8.0);
    gv.setCenter(sf::Vector2f(1800,0));

    GraphViewer::Node &n1 = gv.getNode(route[0]);
    n1.setColor(GraphViewer::GREEN);
    n1.setSize(50.0);
    GraphViewer::Node &n2 = gv.getNode(route.back());
    n2.setColor(GraphViewer::RED);
    n2.setSize(50.0);
    int lastEdge = 500000;
    for(unsigned  a = route.size()-1; a > 0; a--) {
        Node previews = route[a-1];
        Node next = route[a];
        Node node2 = next;
        Node node1 = next;
        while (node2 != previews) {
            node2 = prev.at(previews).at(node2);
            GraphViewer::Edge &e1 = gv.addEdge(lastEdge++, gv.getNode(node2),gv.getNode(node1),GraphViewer::Edge::EdgeType::UNDIRECTED);
            e1.setColor(GraphViewer::BLUE);
            e1.setThickness(0.1);
            node1 = node2;
        }
    }
    return lastEdge;
}