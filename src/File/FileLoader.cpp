#include <fstream>
#include "FileLoader.h"
#include <iostream>
#include <cmath>

const string FileLoader::STRONG_NODES_XY = "../PortoMaps/porto_strong_nodes_xy.txt";

const string FileLoader::STRONG_EDGES = "../PortoMaps/porto_strong_edges.txt";

const string FileLoader::ALL_NODES_XY = "../PortoMaps/porto_full_nodes_xy.txt";

const string FileLoader::ALL_EDGES = "../PortoMaps/porto_full_edges.txt";

const string FileLoader::TAG_NODES_XY = "../Porto/nodes_x_y_porto.txt";

const string FileLoader::TAG_EDGES = "../PortoMaps/edges_porto .txt";

FileLoader::FileLoader() {
    graph = new Graph();
    coordinates = unordered_map<Node, pair<double, double>>();
}

vector<string> FileLoader::split(string &string, char delimiter) {
    std::vector<std::string> res;
    size_t last = 0, next;
    while ((next = string.find(delimiter, last)) != std::string::npos) {
        res.push_back(string.substr(last, next - last));
        last = next + 1;
    }
    res.push_back(string.substr(last));
    return res;
}

void FileLoader::loadNodes(string nodePath, GraphViewer &gv) {
    fstream fs;
    string line;
    bool firstLine = true;
    fs.open(nodePath, ios::in);
    if (!fs.is_open()) {
        cout << "Couldn't read nodes file\n";
        exit(1);
    }
    vector<string> info;
    Node node;
    double x, y;
    int numOfNodes;
    int iteration = 0;
    while (getline(fs, line)) {
        if (firstLine) {
            firstLine = false;
            numOfNodes = stoi(line);
            continue;
        }
        if(iteration%10000==0) printf("Loading Nodes from File: %d%%\n",(iteration*100)/numOfNodes);
        iteration++;
        line.erase(line.begin());
        line.erase(line.end()-1);
        info = split(line, ',');
        x = stod(info[1]);
        y = stod(info[2]);
        node = stoll(info[0]);
        coordinates.insert(make_pair(node, make_pair(x, y)));
        graph->addNode(node);
        GraphViewer::Node &gvNode = gv.addNode(stoll(info[0]), sf::Vector2f(x, y));
        gvNode.setOutlineThickness(0.2);
        gvNode.setSize(0.0001);
        //cout << node << "\t" << x << "\t"<< y << "\n";
        //cout << line <<"\n";

    }
    printf("Node Loading Complete\n\n");
    fs.close();
}

void FileLoader::loadEdges(string edgePath, GraphViewer &gv) {
    fstream fs;
    string line;
    bool firstLine = true;
    fs.open(edgePath, ios::in);
    if (!fs.is_open()) {
        cout << "Couldn't read edge file\n";
        exit(1);
    }
    vector<string> info;
    long long edgeCount = 0;
    Node u, v;
    double dist;
    int numOfNodes;
    int iteration = 0;
    while (getline(fs, line)) {
        if (firstLine) {
            firstLine = false;
            numOfNodes = stoi(line);
            continue;
        }
        if(iteration%10000==0) printf("Loading Edges from File: %d%%\n",(iteration*100)/numOfNodes);
        iteration++;

        line.erase(line.begin());
        line.erase(line.end()-1);
        info = split(line, ',');
        u = stoll(info[0]);
        v = stoll(info[1]);
        dist = pow(pow(coordinates.at(u).first - coordinates.at(v).first, 2) +
                          pow(coordinates.at(u).second - coordinates.at(v).second, 2), 0.5);

        graph->addEdge(u, v, dist/1000/40, edgeCount++);
        GraphViewer::Edge &edge = gv.addEdge(edgeCount-1,gv.getNode(u),gv.getNode(v),GraphViewer::Edge::EdgeType::UNDIRECTED);
        edge.setThickness(2.0);

        //cout << dist << "\n";
        //cout << line <<"\n";
    }
    printf("Edge Loading Complete\n\n");

    fs.close();

}

Graph * FileLoader::getGraph() {
    return graph;
}

const unordered_map<Node, pair<double, double>> & FileLoader::getCoordinates() {
    return coordinates;
}

