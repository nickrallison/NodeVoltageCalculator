#include <iostream>
#include "System.h"

int main()
{
    System sys(3);
    sys.addVoltageSource(1,0,1);
    sys.addImpedance(1,0,1);
    sys.addVCCurrentSource(5,0,2,0,1);
    sys.addImpedance(1,0,2);

    sys.calcVoltages();
}