#ifndef FEUP_CAL_PROJ_MENU_H
#define FEUP_CAL_PROJ_MENU_H

#include <FileLoader.h>
#include <graphviewer.h>
#include <ShortestPathAlgorithm.h>
#include <VaccineRouter.h>
#include <SCCAlgorithm.h>
#include <ClusteringAlgorithm.h>
#include <TSPAlgorithm.h>

class Menu {
public:
    Menu();

    void mainMenu();

private:
    FileLoader loader;

    GraphViewer graphViewer;

    Graph *graph;

    unordered_map<Node, pair<double, double>> coordinates;

    VaccineRouter *vaccineRouter;

    void shortestPathMenu(bool &inMainMenu);

    void singleShortestPathAlgorithmMenu(bool &inMainMenu, bool &inShortestPathMenu);

    void testAllShortestPathAlgorithms();

    void testAllShortestPathAlgorithmsNrandomTimes();

    void executeShortestPath(string algoName, ShortestPathAlgorithm *shortestPathAlgorithm, Node s, Node t);

    void askForShortestPathInput(Node &s, Node &t);

    void reachabilityMenu(bool &inMainMenu);

    void singleDepotToAllClients();

    void singleClientToAllDepots();

    void allPointsOfInterest();

    void connectivityMenu(bool &inMainMenu);

    void validate(unsigned &action, unsigned lim1, unsigned lim2);

    int drawShortestPath(ShortestPathAlgorithm *sp, Node s, Node t, int firstEdge = 500000);

    void eraseDrawnPath(Node s, Node t, int lastEdge);

    void animate(int lastEdge);

    void executeSccAlgorithm(const string &algoName, SCCAlgorithm *sccAlgorithm);

    void singleConnectivityAlgorithmMenu(bool &inMainMenu, bool &inConnectivityMenu);

    void testAllConnectivityAlgorithms();

    void assignMenu(bool &inMainMenu, bool &inVaccineDistributionMenu);

    void showAssignedClients();

    void routingMenu(bool &inMainMenu, bool &inVaccineDistributionMenu);

    void singleRoutingAlgorithmMenu(bool &inMainMenu, bool &inVaccineDistributionMenu, bool &inRoutingMenu);

    void vaccineDistributionMenu(bool &inMainMenu);

    void printRoutes();

    void travellingSalesmanProblemMenu(bool &inMainMenu);

    void singleTravellingSalesmanProblemMenu(bool &inMainMenu, bool &inTravellingSalesmanProblemMenu);

    void executeTravellingSalesmenProblem(string algoName, TSPAlgorithm *tspAlgorithm);

    void getRandomSet(unordered_set<Node> &V, Node &s);

    void testAllTravellingSalesmanProblemAlgorithms();

    void printSccCountStatistics();

    void displayMostPopulatedComponents();

    void testAllVehicleRoutingProblemAlgorithms();

    void displayAllRoutes();

    void displayRoutesForOneDepot();
};

#endif //FEUP_CAL_PROJ_MENU_H
