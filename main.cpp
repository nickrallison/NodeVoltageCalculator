#include <iostream>
#include "System.h"

int main()
{
    System sys(3);
    sys.Resistor(0,1,5);
    sys.Resistor(1,2,5);
    sys.VoltSource(2,0,5);
    //sys.Diode(1, 2, 0.7);
    sys.GND(2);
    sys.FillAll();
    for (int i = 0; i < 4; i++) {
        sys.NodeVoltages(i) = i;
    }
    for (int i = 0; i < 4; i++) {
        sys.UpdateMatrices();

        sys.IterateVoltages();
        sys.UpdateMatrices();
        cout << endl;
        sys.PrintJacobian();
        sys.PrintNewtonFunc();

    }
        sys.PrintVoltages();
        sys.PrintJacobian();
        sys.PrintJacobianInv();
        sys.PrintNewtonFunc();


/*
    sys.PrintNodesConnects();
    sys.PrintIntermediateVecs();
    sys.PrintVoltages();
    sys.PrintJacobianInv();
    sys.PrintJacobian();
    sys.PrintJacobianInv();

*/
    sys.PrintIntermediateVecs();
}