#include <iostream>
#include <Graph.h>
#include <Kosaraju.h>
#include <Tarjan.h>
#include <CapacitatedNearestNeighbour.h>
#include <queue>
#include <File/FileLoader.h>
#include <Dijkstra.h>
#include <MutablePriorityQueue.h>
#include <VaccineRouter.h>
#include <cstdlib>
#include <chrono>
#include <synchapi.h>
#include "graphviewer.h"
#include <Menu.h>

#include <DijkstraBidirectional.h>
/*
 *
 * olha eu nao sei se reparaste mas o mapa do "porto" tem parte de gaia
 */
//using namespace std::chrono;
int main() {
    //time_t t;
    Graph *g = new Graph();
    g->addNode(0);
    g->addNode(1);
    g->addNode(2);
    g->addNode(3);
    g->addNode(4);
    g->addNode(5);
    g->addEdge(0,1,1,0);
    g->addEdge(1,2,2,1);
    g->addEdge(2,3,3,2);
    g->addEdge(3,4,4,3);
    g->addEdge(4,5,1,4);
    DijkstraBidirectional *dijkstraBidirectional = new DijkstraBidirectional(MutablePriorityQueue<NodeWrapper>(), MutablePriorityQueue<NodeWrapper>(),
            g, 0, 5);
    dijkstraBidirectional->execute();
    Menu m = Menu();
    m.mainMenu();
//    int c;
//    cin >> c;
//    auto t = time(NULL);
//    srand(t);
//    cout << t << endl;
//    GraphViewer gv;
//    gv.setScale(8.0);
//    gv.setCenter(sf::Vector2f(1800,0));
//    FileLoader loader;
//    auto start_time = hrc::now();
//
//    loader.loadNodes(FileLoader::ALL_NODES_XY, gv);
//    loader.loadEdges(FileLoader::ALL_EDGES, gv);
    //gv.setZipEdges(true);
//cout << loader.getGraph()->getNodeSet().size()<< endl;
//    VaccineRouter vr(loader.getGraph(), loader.getCoordinates(), new Constraints(2.0, 1000), new RandInfo(4, 60, 100, 49));
    /*for (auto pair : vr.getNodeNumberInEachScc()) {
        if (pair.second > 100)
        cout << pair.first << " " << pair.second << endl;
    }*/
//    int a;
    //cin >> a;
//    cout << endl;
//    cout << "_________________________________________________" << endl;
//    vr.assignKmeans();
//    //vr.printSccCount();
//    //vr.solveNearestNeighbour();
//   vr.solveBellmanHeldKarp();
//
//   cout << "time - "<< std::chrono::duration_cast<std::chrono::seconds>(hrc::now()-start_time).count();
//int d;
//cin >> d;
    //  vr.solveClarkWright();
//  for (Node depot : vr.getDepots()) {
//        cout << "depot " << depot << endl;
//        cout << vr.getRoutes().at(depot).size()<<endl;
//        for (const Route * route : vr.getRoutes().at(depot)) {
//            cout << *route;
//            cout << endl;
//        }
//        cout << "______________________________________" << endl;
//    }

//    vr.mapDrawing(gv);
//    vr.drawAssignedClients(gv);
//    vr.drawMostPopulatedScc(gv);
//    int lastEdge = vr.pathDrawing(gv);
//    int lastEdge = vr.pathDrawingAt(*vr.getDepots().begin(),gv);
//    gv.setEnabledEdgesText(false);

//    gv.createWindow(1600,900);
//    vr.animate(lastEdge,gv);
//    vr.animateInst(lastEdge,gv);
//    gv.setZipEdges(true);
//    gv.join();

    return 0;
}