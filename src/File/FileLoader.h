#ifndef FEUP_CAL_PROJ_FILELOADER_H
#define FEUP_CAL_PROJ_FILELOADER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <Graph.h>
#include "graphviewer.h"


using namespace std;

class FileLoader {
public:
    FileLoader();

    void loadNodes(string nodePath, GraphViewer &gv);

    void loadEdges(string edgePath, GraphViewer &gv);

    Graph * getGraph();

    const unordered_map<Node, pair<double, double>> & getCoordinates();

    static const string STRONG_NODES_XY;

    static const string STRONG_EDGES;

    static const string ALL_NODES_XY;

    static const string ALL_EDGES;

    static const string TAG_NODES_XY;

    static const string TAG_EDGES;

private:
    vector<string> split(string &string, char delimiter);

    unordered_map<Node, pair<double, double>> coordinates;

    Graph * graph;

    unordered_set<Node> clients;

    unordered_set<Node> depots;

};


#endif //FEUP_CAL_PROJ_FILELOADER_H
