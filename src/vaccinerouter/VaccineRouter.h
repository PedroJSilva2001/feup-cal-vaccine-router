#ifndef FEUP_CAL_PROJ_VACCINEROUTER_H
#define FEUP_CAL_PROJ_VACCINEROUTER_H

#include <graphviewer.h>
#include <Graph.h>
#include <ClusteringAlgorithm.h>
#include <VRPAlgorithm.h>
#include <Route.h>
#include <TravellingSalesmenProblem/TSPAlgorithm.h>

struct RandInfo {

    unsigned depotNum;
    unsigned clientNum;
    unsigned demandLowerLimit;
    unsigned demandVariable;

    RandInfo(unsigned depotNum_, unsigned clientNum_,
             unsigned demandLowerLimit_, unsigned demandVariable_) {
        depotNum = depotNum_;
        clientNum = clientNum_;
        demandLowerLimit = demandLowerLimit_;
        demandVariable = demandVariable_;
    }
};

struct Constraints {

    Weight t;
    unsigned C;

    Constraints(Weight t_, unsigned C_) {
        t = t_;
        C = C_;
    }
};

class VaccineRouter {
public:
    VaccineRouter(Graph * graph_, const unordered_set<Node> & depots_, const unordered_set<Node> & clients_,
                  const unordered_map<Node, unsigned> & demand_, const unordered_map<Node, pair<double, double>> & coordinates,
                  Constraints *constraints_);

    VaccineRouter(Graph * graph_, const unordered_map<Node, pair<double, double>> & coordinates_, Constraints *constraints_, RandInfo *randInfo_);

    void assignNaive();

    void assignKmeans();

    void printSccCount();

    void solveClarkWright();

    void solveNearestNeighbour();

    void solveBellmanHeldKarp();

    const unordered_map<Node, vector<Route*>> & getRoutes() const;

    const unordered_set<Node> & getDepots() const;

    const unordered_set<Node> & getClients() const;

    void mapDrawing(GraphViewer &gv);

    int pathDrawing(GraphViewer &gv);

    int pathDrawingAt(Node depot, GraphViewer &gv);

    void animate(int lastEdgeId, GraphViewer &gv);

    void animateInst(int lastEdgeId, GraphViewer &gv);

    const unordered_map<Node, long long> & getNodeNumberInEachScc();

    const unordered_map<Node, vector<Node>> &getReachabilities() const;

    void drawMostPopulatedScc(GraphViewer &gv);

    void hideNodes(GraphViewer &gv);

    void drawAssignedClients(GraphViewer &gv);

    void drawAssignedClientsAt(Node depot, GraphViewer &gv);

    int drawTSP(TSPAlgorithm *tsp, vector<Node> route, GraphViewer &gv);

    int getAssigmentExecutionTime() const;

    int getRoutingExecutionTime() const;

    const unordered_map<Node, unordered_set<Node>> &getAssignedClients() const;

    const unordered_map<Node, unordered_map<Node, Weight>> &getCost() const;

private:
    Graph * graph;

    unordered_set<Node> depots;

    unordered_set<Node> clients;

    unordered_map<Node, unsigned> demand;

    Constraints *constraints;

    RandInfo *randInfo;

    unordered_map<Node, Node> scc;  //nó -> representativo

    unordered_map<Node, unordered_map<Node, Weight>> cost;

    unordered_map<Node, unordered_map<Node, Node>> prev;

    unordered_map<Node, unordered_set<Node>> assignedClients;

    unordered_map<Node, pair<double, double>> coordinates;

    unordered_map<Node, long long> nodeNumberInEachScc;   //representativo -> numero nós

    unordered_map<Node, vector<Route*>> routes;

    vector<Node> nodesInBiggestScc;

    Node biggestSccRepNode;

    unordered_map<Node, vector<Node>> reachable;

    int assignementExecutionTime;

    int routingExecutionTime;

    void computeScc();

    void computeShortestPaths();

    void assignClientsToDepots(ClusteringAlgorithm * clusteringAlgorithm);

    void addRoutes(VRPAlgorithm * vrpAlgorithm, Node depot);

    void setNodeNumberInEachScc();

    void setBiggestSccRepNode();

    void setNodesInBiggestScc();

    void setRandomDepots();

    void setRandomClients();

    void setRandomDemands();

    void voidComputeReachabilities();


};


#endif //FEUP_CAL_PROJ_VACCINEROUTER_H
