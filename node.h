//
// Created by nickr on 2022-06-18.
//

#ifndef NONLINEAREQUATIONSOLVER_NODE_H
#define NONLINEAREQUATIONSOLVER_NODE_H

#include "./Eigen/Dense"
#include <cmath>
#include <vector>


class node {
public:
    int nodeNum;
    int voltage;
    bool attachedToVoltage = false;


    std::vector<std::vector<double>> impedanceVec; //[val, n]
    std::vector<std::vector<double>> voltageVec;   //[val, n]
    std::vector<std::vector<double>> currentVec;   //[val, n]
    std::vector<std::vector<double>> vcvoltageVec; //[val, n, controlledn1, controlledn2]
    std::vector<std::vector<double>> ccvoltageVec; //[val, n, controlledn1, controlledn2]
    std::vector<std::vector<double>> vccurrentVec; //[val, n, controlledn1, controlledn2]
    std::vector<std::vector<double>> cccurrentVec; //[val, n, controlledn1, controlledn2]

    node(int nodeNum) {
        this->nodeNum = nodeNum;
        this->voltage = 0;
    }

    void addImpedance(double Z, int node) {
        std::vector<double> vectemp = {Z, double(node)};
        this->impedanceVec.push_back(vectemp);
    }

    void addVoltageSource(double V, int node) {
        this->attachedToVoltage = true;
        std::vector<double> vectemp = {V, double(node)};
        this->voltageVec.push_back(vectemp);
    }

    void addCurrentSource(double I, int node) {
        std::vector<double> vectemp = {I, double(node)};
        this->currentVec.push_back(vectemp);
    }

    void addVCVoltageSource(double gain, int node, int control0, int control1) {
        this->attachedToVoltage = true;
        std::vector<double> vectemp = {gain, double(node), double(control0), double(control1)};
        this->vcvoltageVec.push_back(vectemp);
    }

    //void addCCVoltageSource(double gain, int node0, int node1, int control0, int control1) {
        //std::vector<double> vectemp = {gain, double(node), double(control0), double(control1)};
        //this->ccvoltageVec.push_back(vectemp);
    //}

    void addVCCurrentSource(double gain, int node, int control0, int control1) {
        std::vector<double> vectemp = {gain, double(node), double(control0), double(control1)};
        this->vccurrentVec.push_back(vectemp);
    }

    //void addCCCurrentSource(int gain, int node0, int node1, int control0, int control1) {
        //std::vector<double> vectemp = {gain, double(node), double(control0), double(control1)};
        //this->cccurrentVec.push_back(vectemp);
    //}

    Eigen::VectorXd returnCurrentEq(int nodes) {
        Eigen::VectorXd out(nodes + 1);
        out.col(0).setZero();
        for (int i = 0; i < this->impedanceVec.size(); i++) { //{Z, double(node)};
            out(this->nodeNum) += 1 / this->impedanceVec[i][0];
            out(int(this->impedanceVec[i][1])) -= 1 / this->impedanceVec[i][0];
        }
                                                            // node0  --->>>  node1      node0 is ref, node 1 is target, current will be positive
        for (int i = 0; i < this->currentVec.size(); i++) { // {I, double(node)}
            out(nodes) -= this->currentVec[i][0];
        }

        for (int i = 0; i < this->vccurrentVec.size(); i++) { // {gain, double(node), double(control0), double(control1)};
            out(int(this->vccurrentVec[i][2])) -= this->vccurrentVec[i][0];
            out(int(this->vccurrentVec[i][3])) += this->vccurrentVec[i][0];

        }

        return out;

    }

};



#endif //NONLINEAREQUATIONSOLVER_NODE_H
