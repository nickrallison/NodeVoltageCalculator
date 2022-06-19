//
// Created by nickr on 2022-04-21.
//

#ifndef NONLINEAREQUATIONSOLVER_SYSTEM_H
#define NONLINEAREQUATIONSOLVER_SYSTEM_H

#include "./Eigen/Dense"
#include <cmath>
#include "node.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class System {
public:
    int nodeNum;

    std::vector<node> nodes;                        //node 0 is ground
    std::vector<std::vector<double>> impedanceVec;     //[val, n1, n2]
    std::vector<std::vector<double>> voltageVec;       //[val, n1, n2]
    std::vector<std::vector<double>> currentVec;       //[val, n1, n2]
    std::vector<std::vector<double>> vcvoltageVec;     //[val, n1, n2, controlledn1, controlledn2]
    //std::vector<std::vector<double>> ccvoltageVec;   //[val, n1, n2, controlledn1, controlledn2]
    std::vector<std::vector<double>> vccurrentVec;     //[val, n1, n2, controlledn1, controlledn2]
    //std::vector<std::vector<double>> cccurrentVec;   //[val, n1, n2, controlledn1, controlledn2]

    System(int nodeNum) {
        this->nodeNum = nodeNum;
        for (int i = 0; i < nodeNum; i++) {
            node temp(i);
            this->nodes.push_back(temp);
        }
    }

    void addImpedance(double Z, int node0, int node1) {
        std::vector<double> vectemp = {Z, double(node0), double(node1)};
        this->impedanceVec.push_back(vectemp);
        this->nodes[node0].addImpedance(Z, node1);
        this->nodes[node1].addImpedance(Z, node0);
    }

    void addVoltageSource(double V, int node0, int node1) { // node0  --- +++  node1
        std::vector<double> vectemp = {V, double(node0), double(node1)};
        this->voltageVec.push_back(vectemp);
        this->nodes[node0].addVoltageSource(V, node1);
        this->nodes[node1].addVoltageSource(-V, node0);
    }

    void addCurrentSource(double I, int node0, int node1) { // node0  --->>>  node1
        std::vector<double> vectemp = {I, double(node0), double(node1)};
        this->currentVec.push_back(vectemp);
        this->nodes[node0].addCurrentSource(I, node1);
        this->nodes[node1].addCurrentSource(-I, node0);
    }

    void addVCVoltageSource(double gain, int node0, int node1, int control0, int control1) {
        std::vector<double> vectemp = {gain, double(node0), double(node1), double(control0), double(control1)};
        this->vcvoltageVec.push_back(vectemp);
        this->nodes[node0].addVCVoltageSource(gain, node1, control0, control1);
        this->nodes[node1].addVCVoltageSource(-gain, node0, control0, control1);
    }

    /*
    void addCCVoltageSource(double gain, int node0, int node1, int control0, int control1) {
        std::vector<double> vectemp = {gain, double(node0), double(node1), double(control0), double(control1)};
        this->ccvoltageVec.push_back(vectemp);
    }
     */

    void addVCCurrentSource(double gain, int node0, int node1, int control0, int control1) {
        std::vector<double> vectemp = {gain, double(node0), double(node1), double(control0), double(control1)};
        this->vccurrentVec.push_back(vectemp);
        this->nodes[node0].addVCCurrentSource(gain, node1, control0, control1);
        this->nodes[node1].addVCCurrentSource(-gain, node0, control0, control1);
    }

    /*
    void addCCCurrentSource(double gain, int node0, int node1, int control0, int control1) {
        std::vector<double> vectemp = {gain, double(node0), double(node1), double(control0), double(control1)};
        this->cccurrentVec.push_back(vectemp);
    }
    */

    void calcVoltages() {
        int rows = 0;
        int cols = this->nodeNum;
        Eigen::MatrixXd mat(rows, cols);
        Eigen::VectorXd vec(rows);

        rows++;                                 // GND Equation
        mat.conservativeResize(rows, cols);
        vec.conservativeResize(rows);
        mat.row(rows - 1).setZero();
        vec.row(rows - 1).setZero();
        mat(rows - 1, 0) = 1;

        for (int i = 0; i < nodeNum; i++) {
            if (!this->nodes[i].attachedToVoltage) {
                rows++;                                 // Current Eqns
                mat.conservativeResize(rows, cols);
                vec.conservativeResize(rows);
                mat.row(rows - 1).setZero();
                vec.row(rows - 1).setZero();

                Eigen::VectorXd tempVec = this->nodes[i].returnCurrentEq(nodeNum);
                for (int j = 0; j < nodeNum; j++) {
                    mat(rows - 1, j) = tempVec(j);
                }
                vec(rows - 1) = tempVec(nodeNum);
            }
        }

        for (int i = 0; i < this->voltageVec.size(); i++) {
            rows++;                                 // Static Voltage Eqn
            mat.conservativeResize(rows, cols);
            vec.conservativeResize(rows);
            mat.row(rows - 1).setZero();
            vec.row(rows - 1).setZero();

            // {V, double(node0), double(node1)};
            // node0  --- +++  node1

            mat(rows - 1, int(this->voltageVec[i][2])) = 1;
            mat(rows - 1, int(this->voltageVec[i][1])) = -1;
            vec(rows - 1) = this->voltageVec[i][0];
        }

        for (int i = 0; i < this->vcvoltageVec.size(); i++) {
            rows++;                                 // Controlled Voltage Eqn
            mat.conservativeResize(rows, cols);
            vec.conservativeResize(rows);
            mat.row(rows - 1).setZero();
            vec.row(rows - 1).setZero();

            // {gain, double(node0), double(node1), double(control0), double(control1)};
            // node0  --- +++  node1

            mat(rows - 1, int(this->vcvoltageVec[i][1])) = 1;
            mat(rows - 1, int(this->vcvoltageVec[i][2])) = -1;
            mat(rows - 1, int(this->vcvoltageVec[i][3])) = -this->vcvoltageVec[i][0];
            mat(rows - 1, int(this->vcvoltageVec[i][4])) = this->vcvoltageVec[i][0];

        }

        for (int i = 0; i < mat.rows(); i++) {
            for (int j = 0; j < mat.cols(); j++) {
                std::cout << mat(i, j) << ' ';
            }
            std::cout << " = " << vec(i) << '\n';
        }
        std::cout << '\n';
        Eigen::VectorXd x = mat.colPivHouseholderQr().solve(vec);
        std::cout << "The solution is:\n" << x << std::endl;
    }

};


#endif //NONLINEAREQUATIONSOLVER_SYSTEM_H
