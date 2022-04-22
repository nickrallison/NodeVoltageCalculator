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

    void printConnect() {
        cout << "Original Node: " << this->node0 << ", Connected Node: " << this->node1 << ", Type: " << this->type << ", Value: " << this->value << endl;
    }
};

class Node {
public:
    int nodeValue;
    vector<connect> connections;
    bool gnd = false;
    bool src = false;
    int diodes = 0;

    Node(int node) {
        this->nodeValue = node;
    }

    void AddResistor(int node, double value) {
        this->connections.push_back(connect(this->nodeValue, node, "R", value));
    }
    void AddDiode(int node, double value, string direction) {
        this->connections.push_back(connect(this->nodeValue, node, direction, value));
        this->diodes++;
    }
    void AddVSource(int node, double value, string direction) {
        this->connections.insert(this->connections.begin(), connect(this->nodeValue, node, direction, value));
    }

    void printConnect() {
        cout << "Node: " << this->nodeValue << endl;
        cout << "Ground: " << this->gnd << endl;
        for (int i = 0; i < this->connections.size(); i++) {
            this->connections[i].printConnect();
        }
        cout << endl;
    }
};


#endif //NONLINEAREQUATIONSOLVER_NODE_H
