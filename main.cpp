#include <iostream>
#include "System.h"

int main()
{
    System sys(4);

    sys.GND(0);
    sys.VoltSource(2,1,5);
    sys.Resistor(0,1,1);
    sys.Resistor(0,1,1);
    sys.Resistor(0,3,1);
    sys.Resistor(1,3,2);
    sys.Diode(3, 2, 1);

    sys.FillAll();
    sys.PrintIntermediateVecs();

    sys.IterateVoltages();


    sys.PrintIntermediateVecs();
    sys.PrintJacobian();
    sys.PrintNewtonFunc();
    sys.PrintVoltages();


/*
    sys.PrintNodesConnects();
    sys.PrintIntermediateVecs();
    sys.PrintVoltages();
    sys.PrintJacobianInv();
    sys.PrintJacobian();
    sys.PrintJacobianInv();

*/

}