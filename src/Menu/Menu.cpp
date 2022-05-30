#include "Menu.h"
#include <iostream>
#include <Dijkstra.h>
#include <DijkstraBidirectional.h>
#include <Astar.h>
#include <Kosaraju.h>
#include <Tarjan.h>
#include <MutablePriorityQueue.h>
#include <TimeAstarHeuristic.h>
#include <synchapi.h>
#include <DepthFirstSearch.h>
#include <NearestNeighbour.h>
#include <BellmanHeldKarp.h>
#include <CostFunction.h>
#include <algorithm>
using namespace std;


Menu::Menu() {
    auto t = time(nullptr);
    srand(t);
    loader = FileLoader();
    loader.loadNodes(FileLoader::ALL_NODES_XY, graphViewer);
    loader.loadEdges(FileLoader::ALL_EDGES, graphViewer);
    graph = loader.getGraph();
    coordinates = loader.getCoordinates();
    vaccineRouter = new VaccineRouter(loader.getGraph(), loader.getCoordinates(),
                                      new Constraints(2.0, 1000), new RandInfo(4, 60, 100, 49));
}

void Menu::mainMenu() {
    bool inMainMenu = true;
    unsigned area;
    while (inMainMenu) {
        cout << "###########################\n";
        cout << " Welcome to VaccineRouter!\n";
        cout << "###########################\n\n";
        cout << "[0] Shortest Path\n";
        cout << "[1] Connectivity\n";
        cout << "[2] Reachability\n";
        cout << "[3] Travelling Salesmen Problem\n";
        cout << "[4] Vaccine distribution\n";
        cout << "[5] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 5);
        cout << endl;
        switch (area) {
            case 0:
                shortestPathMenu(inMainMenu);
                break;
            case 1:
                connectivityMenu(inMainMenu);
                break;
            case 2:
                reachabilityMenu(inMainMenu);
                break;
            case 3:
                travellingSalesmanProblemMenu(inMainMenu);
                break;
            case 4:
                vaccineDistributionMenu(inMainMenu);
                break;
            case 5:
                inMainMenu = false;
                break;

        }
    }
}

void Menu::validate(unsigned int &action, unsigned int lim1, unsigned int lim2) {
    while(cin.fail() || cin.peek() != '\n' || action < lim1 || action > lim2) {
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Please try again: ";
        cin >> action;
    }
    cin.ignore();
}

void Menu::shortestPathMenu(bool & inMainMenu) {
    bool inShortestPathMenu = true;
    unsigned area;
    while (inShortestPathMenu) {
        cout << "[0] Test single shortest path algorithm\n";
        cout << "[1] Test all shortest path algorithms\n";
        cout << "[2] Test all algorithms n random times\n";
        cout << "[3] Go back\n";
        cout << "[4] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 4);
        cout << endl;
        switch (area) {
            case 0:
                singleShortestPathAlgorithmMenu(inMainMenu, inShortestPathMenu);
                break;
            case 1:
                testAllShortestPathAlgorithms();
                break;
            case 2:
                testAllShortestPathAlgorithmsNrandomTimes();
                break;
            case 3:
                inShortestPathMenu = false;
                break;
            case 4: {
                inShortestPathMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }
}

void Menu::singleShortestPathAlgorithmMenu(bool &inMainMenu, bool &inShortestPathMenu) {
    bool inShortestPathAlgorithmsMenu = true;
    unsigned area;
    Node s, t;

    while (inShortestPathAlgorithmsMenu) {
        cout << "[0] Dijsktra's algorithm\n";
        cout << "[1] A*\n";
        cout << "[2] Bidirectional Dijsktra's algorithm\n";
        cout << "[3] Go back\n";
        cout << "[4] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 4);
        cout << endl;
        switch (area) {
            case 0: {
                askForShortestPathInput(s, t);
                executeShortestPath("Dijsktra's algorithm", new Dijkstra(MutablePriorityQueue<NodeWrapper>(), graph, s, t), s, t);
                break;
            }
            case 1: {
                askForShortestPathInput(s, t);
                executeShortestPath("A*", new Astar(MutablePriorityQueue<NodeWrapper>(), graph,
                                                    new TimeAstarHeuristic(t, coordinates, 100), s, t), s, t);
                break;
            }
            case 2: {
                askForShortestPathInput(s, t);
                executeShortestPath("Bidirectional Dijsktra's algorithm", new DijkstraBidirectional(MutablePriorityQueue<NodeWrapper>(),
                        MutablePriorityQueue<NodeWrapper>(), graph, s, t), s, t);
                break;
            }
            case 3: {
                inShortestPathAlgorithmsMenu = false;
                break;
            }
            case 4: {
                inShortestPathAlgorithmsMenu = false;
                inShortestPathMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }

}

void Menu::testAllShortestPathAlgorithms() {
    Node s, t;
    askForShortestPathInput(s, t);
    executeShortestPath("Dijsktra's algorithm", new Dijkstra(MutablePriorityQueue<NodeWrapper>(), graph, s, t), s, t);
    cout << endl;
    executeShortestPath("A*", new Astar(MutablePriorityQueue<NodeWrapper>(), graph, new TimeAstarHeuristic(t, coordinates, 100), s, t), s, t);
    cout << endl;
    executeShortestPath("Bidirectional Dijsktra's algorithm", new DijkstraBidirectional(MutablePriorityQueue<NodeWrapper>(),
                                                                                        MutablePriorityQueue<NodeWrapper>(), graph, s, t), s, t);
}

void Menu::testAllShortestPathAlgorithmsNrandomTimes() {
    unsigned n;
    cout << "Type a positive integer smaller or equal than 200: ";
    cin >> n;
    validate(n, 0, 200);
    cout << endl;
}

void Menu::executeShortestPath(string algoName, ShortestPathAlgorithm *shortestPathAlgorithm, Node s, Node t) {
    cout << algoName << " is running..\n";
    shortestPathAlgorithm->execute();
    cout << algoName << " ran for " << shortestPathAlgorithm->getExecutionTime() << " milliseconds and analyzed "
         << shortestPathAlgorithm->getNodesAnalyzed() << " nodes\n";

    if (shortestPathAlgorithm->getTotalWeight() == INF) {
        cout << "It did not find a path between " << s << " and " << t << endl;
    }
    else {
        cout << "The path between " << s << " and " << t << " has a total time of "<<
        shortestPathAlgorithm->getTotalWeight() << " hours if driving at 40 km/h" <<  endl;
        int lastEdge = drawShortestPath(shortestPathAlgorithm, s, t);
        graphViewer.setEnabledEdgesText(false);

        graphViewer.createWindow(1600,900);
        animate(lastEdge);
        graphViewer.setZipEdges(true);
        graphViewer.join();
        eraseDrawnPath(s,t,lastEdge);
    }
    cout << endl;
}

void Menu::animate(int lastEdge){
    graphViewer.lock();
    for(int i = lastEdge-1;i>=500000;i--){
        if(i%10==0){
            graphViewer.unlock();
            Sleep(10);
            graphViewer.lock();
        }
        GraphViewer::Edge &e = graphViewer.getEdge(i);
        e.setThickness(8.0);
    }
    graphViewer.unlock();
}

void Menu::eraseDrawnPath(Node s, Node t, int lastEdge){
    GraphViewer::Node &n1 = graphViewer.getNode(s);
    n1.setColor(GraphViewer::BLACK);
    n1.setSize(0.0);
    GraphViewer::Node &n2 = graphViewer.getNode(t);
    n2.setColor(GraphViewer::BLACK);
    n2.setSize(0.0);
    graphViewer.setZipEdges(false);
    for(int i = 500000; i < lastEdge; i++){
        graphViewer.removeEdge(i);
    }
    graphViewer.closeWindow();
}

int Menu::drawShortestPath(ShortestPathAlgorithm *sp, Node s, Node t, int firstEdge){
    graphViewer.setScale(8.0);
    graphViewer.setCenter(sf::Vector2f(1800,0));

    GraphViewer::Node &n1 = graphViewer.getNode(s);
    n1.setColor(GraphViewer::GREEN);
    n1.setSize(50.0);
    GraphViewer::Node &n2 = graphViewer.getNode(t);
    n2.setColor(GraphViewer::RED);
    n2.setSize(50.0);

    Node node2 = t;
    Node node1 = t;
    while(node2 != s) {
        node2 = sp->getPrev().at(node2);
        GraphViewer::Edge &e1 = graphViewer.addEdge(firstEdge++, graphViewer.getNode(node2), graphViewer.getNode(node1),GraphViewer::Edge::EdgeType::UNDIRECTED);
        e1.setColor(GraphViewer::CYAN);
        e1.setThickness(0.1);
        node1 = node2;
    }
    return firstEdge;
}

void Menu::askForShortestPathInput(Node &s, Node &t) {
    cout << "Type a source and target node: ";
    while (true) {
        cin >> s >> t;
        if (graph->getNodeSet().find(s) == graph->getNodeSet().end()) {
            cout << "\nSource node doesn't exist. Please try again: ";
        }
        else if (graph->getNodeSet().find(t) == graph->getNodeSet().end()) {
            cout << "\nTarget node doesn't exist. Please try again: ";
        }
        else {
            cout << endl;
            break;
        }
    }
}

void Menu::reachabilityMenu(bool &inMainMenu) {
    bool inReachabilityMenu = true;
    unsigned area;
    while (inReachabilityMenu) {
        cout << "[0] Test single depot to all clients\n";
        cout << "[1] Test single client to all depots\n";
        cout << "[2] Test all points of interest\n";
        cout << "[3] Go back\n";
        cout << "[4] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 4);
        cout << endl;
        switch (area) {
            case 0:
                singleDepotToAllClients();
                break;
            case 1:
                singleClientToAllDepots();
                break;
            case 2:
                allPointsOfInterest();
                break;
            case 3:
                inReachabilityMenu = false;
                break;
            case 4: {
                inReachabilityMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }
}

void Menu::singleDepotToAllClients() {
    cout << "Available depots: " << endl;
    for (Node u : vaccineRouter->getDepots()) {
        cout << u << " ";
    }
    cout << "\nChoose a depot: ";
    Node depot;
    while (true) {
        cin >> depot;
        if (vaccineRouter->getDepots().find(depot) == vaccineRouter->getDepots().end()) {
            cout << "\n Depot doesn't exist. Please try again: ";
        }
        else {
            cout << endl;
            break;
        }
    }
    cout << "Depot " << depot << " can reach the following clients: " << endl;
    for (Node poi : vaccineRouter->getReachabilities().at(depot)) {
        if (vaccineRouter->getClients().find(poi) != vaccineRouter->getClients().end()) {
            cout << poi << " ";
        }
    }
    cout << endl << endl;
}

void Menu::singleClientToAllDepots() {
    cout << "Available clients: " << endl;
    for (Node u : vaccineRouter->getClients()) {
        cout << u << " ";
    }
    cout << "\nChoose a client: ";
    Node client;
    while (true) {
        cin >> client;
        if (vaccineRouter->getClients().find(client) == vaccineRouter->getClients().end()) {
            cout << "\n Client doesn't exist. Please try again: ";
        }
        else {
            cout << endl;
            break;
        }
    }
    cout << "Client " << client << " can reach the following depots: " << endl;
    for (Node poi : vaccineRouter->getReachabilities().at(client)) {
        if (vaccineRouter->getDepots().find(poi) != vaccineRouter->getDepots().end()) {
            cout << poi << " ";
        }
    }
    cout << endl << endl;
}

void Menu::allPointsOfInterest() {
    for (Node client : vaccineRouter->getClients()) {
        cout << "Client " << client << " can reach the following points of interest: " << endl;
        for (Node poi : vaccineRouter->getReachabilities().at(client)) {
            if (vaccineRouter->getClients().find(poi) != vaccineRouter->getClients().end()) {
                cout << "Client " << poi << endl;
            }
            else {
                cout << "Depot " << poi << endl;
            }
        }
        cout << endl;
    }
    for (Node depot : vaccineRouter->getDepots()) {
        cout << "Depot " << depot << " can reach the following points of interest: " << endl;
        for (Node poi : vaccineRouter->getReachabilities().at(depot)) {
            if (vaccineRouter->getClients().find(poi) != vaccineRouter->getClients().end()) {
                cout << "Client " << poi << endl;
            }
        }
        cout << endl;
    }
}

void Menu::connectivityMenu(bool &inMainMenu) {
    bool inConnectivityMenu = true;
    unsigned area;
    while (inConnectivityMenu) {
        cout << "[0] Test single connectivity algorithm\n";
        cout << "[1] Test all connectivity algorithms\n";
        cout << "[2] See most populated strongly connected components\n";
        cout << "[3] See strongly connected components statistics\n";
        cout << "[4] Go back\n";
        cout << "[5] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 5);
        cout << endl;
        switch (area) {
            case 0:
                singleConnectivityAlgorithmMenu(inMainMenu, inConnectivityMenu);
                break;
            case 1:
                testAllConnectivityAlgorithms();
                break;
            case 2:
                displayMostPopulatedComponents();
                break;
            case 3:
                printSccCountStatistics();
            case 4:
                inConnectivityMenu = false;
                break;
            case 5: {
                inConnectivityMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }
}

void Menu::executeSccAlgorithm(const string &algoName, SCCAlgorithm *sccAlgorithm) {
    cout << algoName << " is running..\n";
    sccAlgorithm->execute();
    cout << algoName << " ran for " << sccAlgorithm->getExecutionTime() << " milliseconds\n\n";
}

void Menu::singleConnectivityAlgorithmMenu(bool &inMainMenu, bool &inConnectivityMenu) {
    bool inConnectivityAlgorithmsMenu = true;
    unsigned area;
    while (inConnectivityAlgorithmsMenu) {
        cout << "[0] Kosaraju's algorithm\n";
        cout << "[1] Tarjan's algorithm\n";
        cout << "[2] Go back\n";
        cout << "[3] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 3);
        cout << endl;
        switch (area) {
            case 0: {
                executeSccAlgorithm("Kosaraju's algorithm", new Kosaraju(graph));
                break;
            }
            case 1: {
                executeSccAlgorithm("Tarjan's algorithm", new Tarjan(graph));
                break;
            }
            case 2: {
                inConnectivityAlgorithmsMenu = false;
                break;
            }
            case 3: {
                inConnectivityAlgorithmsMenu = false;
                inConnectivityMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }
}


void Menu::testAllConnectivityAlgorithms() {
    executeSccAlgorithm("Kosaraju's algorithm", new Kosaraju(graph));
    cout << endl;
    executeSccAlgorithm("Tarjan's algorithm", new Kosaraju(graph));
    cout << endl;
}

void Menu::assignMenu(bool &inMainMenu, bool &inVaccineDistributionMenu) {
    bool inAssignMenu = true;
    unsigned area;

    while (inAssignMenu) {
        cout << "[0] Assign clients naively\n";
        cout << "[1] Assign clients with K means\n";
        cout << "[2] See assigned clients\n";
        cout << "[3] Go back\n";
        cout << "[4] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 3);
        cout << endl;
        switch (area) {
            case 0: {
                cout << "Clients are being assigned to depots naively..\n";
                vaccineRouter->assignNaive();
                cout << "Naive assignement ran for " << vaccineRouter->getAssigmentExecutionTime() << " milliseconds" << endl << endl;
                break;
            }
            case 1: {
                cout << "Clients are being assigned to depots with K means..\n";
                vaccineRouter->assignKmeans();
                cout << "K means assignement ran for " << vaccineRouter->getAssigmentExecutionTime() << " milliseconds" << endl << endl;
                break;
            }
            case 2: {
                showAssignedClients();
                break;
            }
            case 3: {
                inAssignMenu = false;
                break;
            }
            case 4: {
                inVaccineDistributionMenu = false;
                inAssignMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }
}

void Menu::showAssignedClients() {
    for (Node depot : vaccineRouter->getDepots()) {
        cout << "Depot " << depot << " is assigned to the following clients" << endl;
        for (Node client : vaccineRouter->getAssignedClients().at(depot)) {
            cout << client << endl;
        }
        cout << endl;
    }
    graphViewer.setScale(8.0);
    graphViewer.setCenter(sf::Vector2f(1800,0));
    vaccineRouter->drawAssignedClients(graphViewer);
    graphViewer.setEnabledEdgesText(false);
    graphViewer.createWindow(1600,900);
    graphViewer.join();
    vaccineRouter->hideNodes(graphViewer);
    graphViewer.closeWindow();
}

void Menu::routingMenu(bool &inMainMenu, bool &inVaccineDistributionMenu) {
    bool inRoutingMenu = true;
    unsigned area;

    while (inRoutingMenu) {
        cout << "[0] Test routing algorithm\n";
        cout << "[1] Test all routing algorithms\n";
        cout << "[2] Visualize all routes\n";
        cout << "[3] Visualize routes just for one depot\n";
        cout << "[4] Go back\n";
        cout << "[5] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 5);
        cout << endl;
        switch (area) {
            case 0:
                singleRoutingAlgorithmMenu(inMainMenu, inVaccineDistributionMenu, inRoutingMenu);
                break;
            case 1: {
                testAllVehicleRoutingProblemAlgorithms();
                break;
            }
            case 2: {
                displayAllRoutes();
                break;
            }
            case 3: {
                displayRoutesForOneDepot();
                break;
            }
            case 4:
                inRoutingMenu = false;
                break;
            case 5:
                inRoutingMenu = false;
                inVaccineDistributionMenu = false;
                inMainMenu = false;
                break;
        }
    }
}

void Menu::singleRoutingAlgorithmMenu(bool &inMainMenu, bool &inVaccineDistributionMenu, bool &inRoutingMenu) {
    bool inRoutingAlgorithmMenu = true;
    unsigned area;

    while (inRoutingAlgorithmMenu) {
        cout << "[0] Clark-Wright savings algorithm\n";
        cout << "[1] Bellman-Held-Karp\n";
        cout << "[2] Nearest Neighbour\n";
        cout << "[3] Go back\n";
        cout << "[4] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 4);
        cout << endl;
        switch (area) {
            case 0: {
                cout << "Routing being done by savings algorithm.." << endl;
                vaccineRouter->solveClarkWright();
                cout << "Savings algorithm ran for " << vaccineRouter->getRoutingExecutionTime() << " milliseconds" << endl;
                printRoutes();
                break;
            }
            case 1: {
                cout << "Routing being done by Bellman-Held-Karp algorithm.." << endl;
                vaccineRouter->solveBellmanHeldKarp();
                cout << "Bellman-Held-Karp ran for " << vaccineRouter->getRoutingExecutionTime() << " milliseconds" << endl;
                printRoutes();
                break;
            }
            case 2: {
                cout << "Routing being done by Nearest Neighbour algorithm.." << endl;
                vaccineRouter->solveNearestNeighbour();
                cout << "Nearest neighbour ran for " << vaccineRouter->getRoutingExecutionTime() << " milliseconds" << endl;
                printRoutes();
                break;
            }
            case 3: {
                inRoutingAlgorithmMenu = false;
                break;
            }
            case 4: {
                inRoutingAlgorithmMenu = false;
                inRoutingMenu = false;
                inVaccineDistributionMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }
}

void Menu::vaccineDistributionMenu(bool &inMainMenu) {
    bool inVaccineDistributionMenu = true;
    unsigned area;

    while (inVaccineDistributionMenu) {
        cout << "[0] Assign\n";
        cout << "[1] Route\n";
        cout << "[2] Go back\n";
        cout << "[3] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 4);
        cout << endl;
        switch (area) {
            case 0: {
                assignMenu(inMainMenu, inVaccineDistributionMenu);
                break;
            }
            case 1: {
                routingMenu(inMainMenu, inVaccineDistributionMenu);
                break;
            }
            case 2: {
                inVaccineDistributionMenu = false;
                break;
            }
            case 3: {
                inVaccineDistributionMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }

}

void Menu::printRoutes() {
    Weight total = 0;
    for (Node depot : vaccineRouter->getDepots()) {
        cout << "______________________________________" << endl;
        cout << "Routes for Depot " << depot << endl;
        cout << "Route number: " << vaccineRouter->getRoutes().at(depot).size()<<endl;
        for (const Route * route : vaccineRouter->getRoutes().at(depot)) {
            total += route->getFullRouteWeight();
            cout << *route;
            cout << endl;
        }
    }
    cout << "Total vaccine delivery time in hours: " << total << endl << endl;
}

void Menu::travellingSalesmanProblemMenu(bool &inMainMenu) {
    bool inTravellingSalesmanProblemMenu = true;
    unsigned area;
    while (inTravellingSalesmanProblemMenu) {
        cout << "[0] Test single travelling salesman problem algorithm\n";
        cout << "[1] Test all travelling salesman problem algorithms\n";
        cout << "[2] Go back\n";
        cout << "[3] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 4);
        cout << endl;
        switch (area) {
            case 0:
                singleTravellingSalesmanProblemMenu(inMainMenu, inTravellingSalesmanProblemMenu);
                break;
            case 1:
                testAllTravellingSalesmanProblemAlgorithms();
                break;
            case 2:
                inTravellingSalesmanProblemMenu = false;
                break;
            case 4: {
                inTravellingSalesmanProblemMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }
}

void Menu::singleTravellingSalesmanProblemMenu(bool &inMainMenu, bool &inTravellingSalesmanProblemMenu) {
    bool inTravellingSalesmanProblemAlgorithmsMenu = true;
    unsigned area;
    unordered_set<Node> V = unordered_set<Node>();
    Node s;
    getRandomSet(V, s);
    while (inTravellingSalesmanProblemAlgorithmsMenu) {
        cout << "[0] Bellman-Held-Karp\n";
        cout << "[1] Nearest Neighbour\n";
        cout << "[2] Go back\n";
        cout << "[3] Exit\n";
        cout << "Choose the corresponding number: ";
        cin >> area;
        validate(area, 0, 4);
        cout << endl;
        switch (area) {
            case 0: {
                executeTravellingSalesmenProblem("Bellman-Held-Karp", new BellmanHeldKarp(V, s, new CostFunction(V.size(), vaccineRouter->getCost())));
                break;
            }
            case 1: {
                executeTravellingSalesmenProblem("Nearest Neighbour", new NearestNeighbour(V, s, new CostFunction(V.size(), vaccineRouter->getCost())));
                break;
            }
            case 2: {
                inTravellingSalesmanProblemAlgorithmsMenu = false;
                break;
            }
            case 3: {
                inTravellingSalesmanProblemAlgorithmsMenu = false;
                inTravellingSalesmanProblemMenu = false;
                inMainMenu = false;
                break;
            }
        }
    }
}


void Menu::executeTravellingSalesmenProblem(string algoName, TSPAlgorithm *tspAlgorithm) {
    cout << algoName << " is running..\n";
    tspAlgorithm->execute();
    cout << algoName << " ran for " << tspAlgorithm->getExecutionTime()<< " milliseconds" << endl;
    Weight routeDuration = 0;
    vector<Node> route = tspAlgorithm->getRoute();
    cout << "Path: ";
    for (unsigned i = 0; i < route.size(); i++) {
        if (i != tspAlgorithm->getRoute().size()-1) {
            routeDuration += vaccineRouter->getCost().at(route[i]).at(route[i+1]);
        }
        cout << route[i];
        if (i != route.size()-1) {
            cout << " -> ";
        }
    }
    printf("GOT HERE!\n");
    int lastEdge = vaccineRouter->drawTSP(tspAlgorithm,route,graphViewer);
    graphViewer.setEnabledEdgesText(false);
    graphViewer.createWindow(1600, 900);
    vaccineRouter->animate(lastEdge,graphViewer);
    graphViewer.setZipEdges(true);
    graphViewer.join();
    eraseDrawnPath(route[0],route.back(),lastEdge);
}

void Menu::getRandomSet(unordered_set<Node> &V, Node &s) {
    vector<Node> depots = vector<Node>(vaccineRouter->getDepots().begin(), vaccineRouter->getDepots().end());
    vector<Node> clients = vector<Node>(vaccineRouter->getClients().begin(), vaccineRouter->getClients().end());
    size_t TSP_MAX_N = 13;
    size_t size = min(TSP_MAX_N, clients.size());
    while (V.size() != size) {
        V.insert(clients.at(rand()%clients.size()));
    }
    Node depot = depots.at(rand()%depots.size());
    V.insert(depot);
    cout << V.size()<< endl;
    s = depot;
}

void Menu::testAllTravellingSalesmanProblemAlgorithms() {
    unordered_set<Node> V = unordered_set<Node>();
    Node s;
    getRandomSet(V, s);
    CostFunctionTSP *costFunctionTsp = new CostFunction(V.size(), vaccineRouter->getCost());
    executeTravellingSalesmenProblem("Bellman-Held-Karp", new BellmanHeldKarp(V, s, costFunctionTsp));
    cout << endl;
    executeTravellingSalesmenProblem("Nearest Neighbour", new NearestNeighbour(V, s, costFunctionTsp));
}

void Menu::displayMostPopulatedComponents() {
    graphViewer.setScale(8.0);
    graphViewer.setCenter(sf::Vector2f(1800,0));
    vaccineRouter->drawMostPopulatedScc(graphViewer);
    graphViewer.setEnabledEdgesText(false);
    graphViewer.createWindow(1600,900);
    graphViewer.join();
    vaccineRouter->hideNodes(graphViewer);
    graphViewer.closeWindow();
}

void Menu::printSccCountStatistics() {
    long long POPULATED_N = 100;
    long long biggestCount = 0;
    double averageCount = 0;
    cout << "Strongly connected component node counts bigger than 100:\n";
    for (const pair<Node, long long> sccCountPair : vaccineRouter->getNodeNumberInEachScc()) {
        if (sccCountPair.second > POPULATED_N) {
            cout << sccCountPair.second << endl;
        }
        if (biggestCount < sccCountPair.second) {
            biggestCount = sccCountPair.second;
        }
        averageCount += sccCountPair.second;
    }
    averageCount /= vaccineRouter->getNodeNumberInEachScc().size();
    cout << endl;
    cout << "Number of strongly connected components: " << vaccineRouter->getNodeNumberInEachScc().size() << endl;
    cout << "Node number in biggest component: " << biggestCount << endl;
    cout << "Average node number in a strongly connected component: " << averageCount << endl << endl;
}

void Menu::testAllVehicleRoutingProblemAlgorithms() {
    cout << "Routing being done by savings algorithm.." << endl;
    vaccineRouter->solveClarkWright();
    cout << "Savings algorithm ran for " << vaccineRouter->getRoutingExecutionTime() << " milliseconds" << endl << endl;
    printRoutes();
    cout << "Routing being done by Bellman-Held-Karp algorithm.." << endl;
    vaccineRouter->solveBellmanHeldKarp();
    cout << "Bellman-Held-Karp ran for " << vaccineRouter->getRoutingExecutionTime() << " milliseconds" << endl << endl;
    printRoutes();
    cout << "Routing being done by Nearest Neighbour algorithm.." << endl;
    vaccineRouter->solveNearestNeighbour();
    cout << "Nearest neighbour ran for " << vaccineRouter->getRoutingExecutionTime() << " milliseconds" << endl << endl;
    printRoutes();
}

void Menu::displayAllRoutes() {
    graphViewer.setScale(8.0);
    graphViewer.setCenter(sf::Vector2f(1800,0));
    vaccineRouter->drawAssignedClients(graphViewer);
    int lastEdge = vaccineRouter->pathDrawing(graphViewer);
    graphViewer.setEnabledEdgesText(false);
    graphViewer.createWindow(1600, 900);
    vaccineRouter->animateInst(lastEdge, graphViewer);
    graphViewer.setZipEdges(true);
    graphViewer.join();
    eraseDrawnPath(10, 11, lastEdge);
    vaccineRouter->hideNodes(graphViewer);
}

void Menu::displayRoutesForOneDepot() {
    cout << "Available depots: " << endl;
    for (Node u : vaccineRouter->getDepots()) {
        cout << u << " ";
    }
    cout << "\nChoose a depot: ";
    Node depot;
    while (true) {
        cin >> depot;
        if (vaccineRouter->getDepots().find(depot) == vaccineRouter->getDepots().end()) {
            cout << "\n Depot doesn't exist. Please try again: ";
        }
        else {
            cout << endl;
            break;
        }
    }
    graphViewer.setScale(8.0);
    graphViewer.setCenter(sf::Vector2f(1800,0));
    vaccineRouter->drawAssignedClientsAt(depot, graphViewer);
    int lastE = vaccineRouter->pathDrawingAt(depot, graphViewer);
    graphViewer.setEnabledEdgesText(false);
    graphViewer.createWindow(1600, 900);
    vaccineRouter->animate(lastE,graphViewer);
    graphViewer.setZipEdges(true);
    graphViewer.join();
    eraseDrawnPath(10, 11, lastE);
    vaccineRouter->hideNodes(graphViewer);
}