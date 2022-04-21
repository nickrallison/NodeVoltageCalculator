//
// Created by nickr on 2022-04-21.
//

#ifndef NONLINEAREQUATIONSOLVER_NODE_H
#define NONLINEAREQUATIONSOLVER_NODE_H

using namespace std;

#include <vector>

class connect {
public:
    int node0;
    int node1;
    string type;
    double value;

    connect(int node0, int node1, string type, double value) {
        this->node0 = node0;
        this->node1 = node1;
        this->type = type;
        this->value = value;
    }
};

class Node {
public:
    int nodeValue;
    vector<connect> connections;
    bool gnd = false;
    bool src = false;

    Node(int node) {
        this->nodeValue = node;
    }

    void AddResistor(int node, double value) {
        this->connections.push_back(connect(this->nodeValue, node, "R", value));
    }
    void AddDiode(int node, double value, string direction) {
        this->connections.push_back(connect(this->nodeValue, node, direction, value));
    }
    void AddVSource(int node, double value, string direction) {
        this->connections.push_back(connect(this->nodeValue, node, direction, value));
    }

    void printConnect() {
        cout << "Node: " << this->nodeValue << endl;
        cout << "Ground: " << this->gnd << endl;
        for (int i = 0; i < this->connections.size(); i++) {
            cout << "Connected Node: " << this->connections[i].node1 << ", Type: " << this->connections[i].type << ", Value: " << this->connections[i].value << endl;
        }
        cout << endl;
    }
};


#endif //NONLINEAREQUATIONSOLVER_NODE_H
