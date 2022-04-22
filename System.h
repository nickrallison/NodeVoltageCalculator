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

    void UpdateMatrices() {
        calcNewtonFunc();
        calcJacobian();
    }

    int IterateVoltages() {
        this->UpdateMatrices();
        VectorXd delta = this->JacobianInv * this->NewtonFunc;
        this->NodeVoltages = this->NodeVoltages - delta;
        for (int i = 0; i < nodes; i++) {
            if (!nodeVec[i].src && nodeVec[i].diodes) {
                for (int j = 0; j < nodeVec[i].connections.size(); j++) {
                    PrintIntermediateVecs();
                    if (this->DiodeLogic(nodeVec[i].connections[j])) {
                        nodeVec[i].connections.erase(nodeVec[i].connections.begin() + j);
                        FillAll();
                        return IterateVoltages();
                    }
                }
            }
        }

        for (int i = 0; i < this->GNDVec.size(); i++) {
            this->NodeVoltages(this->GNDVec[i]) = 0;
        }
        for (int i = 0; i < this->VSVec.size(); i++) {
            this->NodeVoltages(this->VSVec[i].node1) = this->NodeVoltages(this->VSVec[i].node0) + this->VSVec[i].value;
        }
        return 0;
    }

    void calcNewtonFunc() {
        int count = 0;

        for (int i = 0; i < this->GNDVec.size(); i++) {
            int value = NodeVoltages(GNDVec[i]);
            this->NewtonFunc(count) = value;
            count++;
        }
        for (int i = 0; i < this->VSVec.size(); i++) {
            int value = NodeVoltages(VSVec[i].node1) - NodeVoltages(VSVec[i].node0) - VSVec[i].value;
            this->NewtonFunc(count) = value;
            count++;
        }

        for (int i = 0; i < this->RDVec.size(); i++) {
            this->NewtonFunc(count) = 0;
            for (int j = 0; j < this->RDVec[i].size(); j++) {
                double value = evaluate(this->RDVec[i][j]);
                this->NewtonFunc(count) += value;
            }
            count++;
        }
    }

    void calcJacobian() {
        int countToV = this->GNDVec.size();
        int countToRD = countToV + this->VSVec.size();

        for (int i = 0; i < this->nodes; i++) {
            for (int j = 0; j < this->nodes; j++) {
                double value = 0;
                if (i < countToV) {                             //GND Levels of jacobian
                    value += this->derivativeGND(j);
                }

                if (countToV <= i && i < countToRD) {           //SRC Levels of Jacobian
                    value += this->derivativeV(this->VSVec[i - countToV], j);
                }
                if (countToRD <= i) {
                    for (int k = 0; k < this->RDVec[i - countToRD].size(); k++) {           //Impedance Level of Jacobian
                        if (this->RDVec[i - countToRD][k].type == "R") {
                            value += derivativeRD(this->RDVec[i - countToRD][k], j);
                        }
                    }
                }
                this->Jacobian(i, j) = value;
            }
        }

        this->JacobianInv = this->Jacobian.inverse();
    }


    double evaluate(connect connection) {
        double deltaV = (this->NodeVoltages(connection.node0) - this->NodeVoltages(connection.node1));

        if (connection.type == "R") {
            return (deltaV / connection.value);
        }
    }

    double derivativeGND(int node) {
        if (node == this->GNDVec[0]) {
            return 1;
        }
        return 0;
    }

    double derivativeV(connect connection, int node) {
        if (node != connection.node0 && node != connection.node1) {
            return 0;
        }
        int value = 1;
        if (node == connection.node0 && connection.value >= 0 || node == connection.node1 && connection.value <= 0) {
            value *= -1;
        }
        return value;
    }

    double derivativeRD(connect connection, int node) {
        if (node != connection.node0 && node != connection.node1) {
            return 0;
        }

        if (connection.type == "R") {
            if (connection.node0 == node) {
                return (1 / connection.value);
            }
            return (-1 / connection.value);
        }

    }


    void FillGNDVec() {
        this->GNDVec.clear();
        for (int i = 0; i < nodes; i++) {
            if (this->nodeVec[i].gnd) {
                this->GNDVec.push_back(i);
            }
        }
    }

    void FillVSRCVec() {
        this->VSVec.clear();
        for (int i = 0; i < nodes; i++) {
            for (int j = 0; j < this->nodeVec[i].connections.size(); j++) {
                if (this->nodeVec[i].connections[j].type == "FVS") {
                    this->VSVec.push_back(this->nodeVec[i].connections[j]);
                }
            }
        }
    }

    void FillRDVec() {
        this->RDVec.clear();
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

    int DiodeLogic(connect diodeConnection) {
        double deltaVoltage = this->NodeVoltages(diodeConnection.node0) - this->NodeVoltages(diodeConnection.node1);
        if (diodeConnection.type == "FD" && deltaVoltage < diodeConnection.value) {
            this->VoltSource(diodeConnection.node1, diodeConnection.node0, -diodeConnection.value);
            return 1;
        }
        if (diodeConnection.type == "RD" && deltaVoltage > -diodeConnection.value) {
            this->VoltSource(diodeConnection.node0, diodeConnection.node1, -diodeConnection.value);
            return 1;
        }
        return 0;
    }

    void Resistor(int node0, int node1, double value) {
        this->nodeVec[node0].AddResistor(node1, value);
        this->nodeVec[node1].AddResistor(node0, value);
    }

    void Diode(int node0, int node1, double value) {
        this->nodeVec[node0].AddDiode(node1, value, "FD");
        this->nodeVec[node1].AddDiode(node0, value, "RD");
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

    void PrintJacobian() {
        cout << this->Jacobian << endl << endl;
    }

    void PrintJacobianInv() {
        cout << this->JacobianInv << endl << endl;
    }


};


#endif //NONLINEAREQUATIONSOLVER_SYSTEM_H
