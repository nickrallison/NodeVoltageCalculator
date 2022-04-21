#include <iostream>
#include "System.h"

int main()
{
    System sys(4);
    sys.Resistor(0,3,5);
    sys.Resistor(2,3,5);
    sys.VoltSource(0,1,1);
    sys.Diode(1, 2, 0.7);
    sys.GND(0);
    sys.FillAll();
    sys.PrintNodesConnects();
    sys.PrintIntermediateVecs();
    sys.PrintVoltages();
}