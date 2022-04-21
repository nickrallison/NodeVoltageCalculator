//
// Created by nickr on 2022-04-21.
//

#ifndef NONLINEAREQUATIONSOLVER_SYSTEM_H
#define NONLINEAREQUATIONSOLVER_SYSTEM_H

#include "./Eigen/Dense"
#include "Node.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class System {
public:
    int nodes;
    VectorXd NodeVoltages;
    vector<Node> nodeVec;
    vector<int> GNDVec;
    vector<connect> VSVec;
    vector<vector<connect>> RDVec;

    System(int nodeNum) {
        this->nodes = nodeNum;
        for (int i = 0; i < nodeNum; i++) {
            this->nodeVec.push_back(Node(i));
        }
        this->NodeVoltages.resize(nodeNum,1);
        for (int i = 0; i < i < this->nodeVec.size(); i++) {
            this->NodeVoltages(i) = 0;
        }
    }


    void FillGNDVec() {
        for (int i = 0; i < this->nodeVec.size(); i++) {
            if (this->nodeVec[i].gnd) {
                this->GNDVec.push_back(i);
            }
        }
    }

    void FillVSRCVec() {
        for (int i = 0; i < this->nodeVec.size(); i++) {
            for (int j = i; j < this->nodeVec[i].connections.size(); j++) {
                if (this->nodeVec[i].connections[j].type == "FVS") {
                    this->VSVec.push_back(this->nodeVec[i].connections[j]);
                }
            }
        }
    }

    void FillRDVec() {
        vector<connect> intermediateVec;
        for (int i = 0; i < this->nodeVec.size(); i++) {
            intermediateVec.clear();
            if (!(this->nodeVec[i].src)) {
                cout << i << endl;
                vector<connect> intermediateVec = {};
                for (int j = 0; j < this->nodeVec[i].connections.size(); j++) {
                    intermediateVec.push_back(this->nodeVec[i].connections[j]);
                }
                this->RDVec.push_back(intermediateVec);
            }
        }
    }

    void FillAll() {
        FillGNDVec();
        FillVSRCVec();
        FillRDVec();
    }


    /*      f(X) vec
     *      First is Gnd
     *      Then we have V sources
     *      Then we append R and Diode Eqs
     */



    void Resistor(int node0, int node1, double value) {
        this->nodeVec[node0].AddResistor(node1, value);
        this->nodeVec[node1].AddResistor(node0, value);
    }

    void Diode(int node0, int node1, double value) {
        this->nodeVec[node0].AddDiode(node1, 1 / exp(value / 0.026 - 1), "FD");
        this->nodeVec[node1].AddDiode(node0, 1 / exp(value / 0.026 - 1), "RD");
    }

    void VoltSource(int node0, int node1, double value) {
        this->nodeVec[node0].AddVSource(node1, value, "FVS");
        this->nodeVec[node1].AddVSource(node0, value, "RVS");
        this->nodeVec[node0].src = true;
        this->nodeVec[node1].src = true;
    }

    void GND(int node) {
        this->nodeVec[node].gnd = true;
        this->nodeVec[node].src = true;
    }



    void PrintNodesConnects() {
        for (int i = 0; i < this->nodeVec.size(); i++) {
            nodeVec[i].printConnect();
        }
    }

    void PrintIntermediateVecs() {
        cout <<"GND Vec: " << endl;
        for (int i = 0; i < this->GNDVec.size(); i++) {
            cout << "Node: " << this->GNDVec[i] << endl;
        }
        cout << endl;

        cout <<"VS Vec: " << endl;
        for (int i = 0; i < this->VSVec.size(); i++) {
            cout << "PrimeNode: " << this->VSVec[i].node0 << ", SecNode: " << this->VSVec[i].node1 << ", Type: " << this->VSVec[i].type << ", Value: " << this->VSVec[i].value << endl;
        }
        cout << endl;

        cout <<"RD Vec: " << endl;
        for (int i = 0; i < this->RDVec.size(); i++) {
            for (int j = 0; j < this->RDVec[i].size(); j++) {
                cout << "PrimeNode: " << this->RDVec[i][j].node0 << ", SecNode: " << this->RDVec[i][j].node1 << ", Type: " << this->RDVec[i][j].type << ", Value: " << this->RDVec[i][j].value << endl;
            }
            cout << endl;
        }
        cout << endl;
    }

    void PrintVoltages() {
        for (int i = 0; i < this->nodeVec.size(); i++) {

        }
    }
};


#endif //NONLINEAREQUATIONSOLVER_SYSTEM_H
