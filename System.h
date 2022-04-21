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

    vector<Node> nodeVec;
    vector<int> GNDVec;
    vector<connect> VSVec;
    vector<vector<connect>> RDVec;

    VectorXd NodeVoltages;
    MatrixXd Jacobian;
    MatrixXd JacobianInv;
    VectorXd NewtonFunc;

    System(int nodeNum) {
        this->nodes = nodeNum;
        for (int i = 0; i < nodeNum; i++) {
            this->nodeVec.push_back(Node(i));
        }

        this->NodeVoltages.resize(nodeNum);
        this->Jacobian.resize(nodeNum,nodeNum);
        this->JacobianInv.resize(nodeNum,nodeNum);
        this->NewtonFunc.resize(nodeNum);

        for (int i = 0; i < nodes; i++) {
            this->NodeVoltages(i) = 0;
        }

    }

    void IterateVoltages() {
        calcNewtonFunc();
        calcJacobian();
        this->NodeVoltages = this->NodeVoltages - this->JacobianInv * this->NewtonFunc;
    }

    void calcNewtonFunc() {
        int count = 0;

        for (int i = 0; i < this->GNDVec.size(); i++) {
            this->NewtonFunc(count) = NodeVoltages(GNDVec[i]);
            count++;
        }

        for (int i = 0; i < this->VSVec.size(); i++) {
            this->NewtonFunc(count) = NodeVoltages(VSVec[i].node1) - NodeVoltages(VSVec[i].node0) - VSVec[i].value;
            count++;
        }


        for (int i = 0; i < this->RDVec.size(); i++) {
            this->NewtonFunc(count) = 0;
            for (int j = 0; j < this->RDVec[i].size(); j++) {
                this->NewtonFunc(count) += evaluate(this->RDVec[i][j]);
                count++;
            }
            count++;
        }
    }

    void calcJacobian() {

    }

    double evaluate(connect connection) {

        if (connection.type == "R") {
            return (this->NodeVoltages(connection.node0) - this->NodeVoltages(connection.node1)) * connection.value;
        }
        if (connection.type == "RD") {
            return connection.value * (exp(-(this->NodeVoltages(connection.node0) - this->NodeVoltages(connection.node1)) / 0.026) - 1);
        }
        if (connection.type == "FD") {
            return connection.value * (exp((this->NodeVoltages(connection.node0) - this->NodeVoltages(connection.node1)) / 0.026) - 1);
        }

    }


    void FillGNDVec() {
        for (int i = 0; i < nodes; i++) {
            if (this->nodeVec[i].gnd) {
                this->GNDVec.push_back(i);
            }
        }
    }

    void FillVSRCVec() {
        for (int i = 0; i < nodes; i++) {
            for (int j = i; j < this->nodeVec[i].connections.size(); j++) {
                if (this->nodeVec[i].connections[j].type == "FVS") {
                    this->VSVec.push_back(this->nodeVec[i].connections[j]);
                }
            }
        }
    }

    void FillRDVec() {
        vector<connect> intermediateVec;
        for (int i = 0; i < nodes; i++) {
            intermediateVec.clear();
            if (!(this->nodeVec[i].src)) {
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
        for (int i = 0; i < nodes; i++) {
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
        for (int i = 0; i < nodes; i++) {
            cout << "Voltage " << i << ": " << this->NodeVoltages(i) << endl;
        }
        cout << endl;
    }

    void PrintNewtonFunc() {
        for (int i = 0; i < nodes; i++) {
            cout << "Value " << i << ": " << this->NewtonFunc(i) << endl;
        }
        cout << endl;
    }
};


#endif //NONLINEAREQUATIONSOLVER_SYSTEM_H
