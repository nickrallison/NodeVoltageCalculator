# NodeVoltageCalculator

## What it does:
It is designed to a flexible implementation of a node voltage calculator implemented in C++, the goal was to compile this down to TI 84 ASM. 
The implementation uses the C++ Eigen library for flexible matrices and easy efficient solving.
Eigen does not like being compiled to Ti 84 ASM due to the use of multiple standard libraries that are not available.
This project is complete and implemented on a desktop, and on hold for a TI 84.

## What I learned:
[Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page "Eigen Homepage") - C++ Linear Algebra Library for solving variable complex systems of equations

[CE Toolchain](https://ce-programming.github.io/toolchain/ "CE Toolchain Homepage") - A tool that is used to compile C++ to assembly used on the TI series of calculators and needed for more complex projects

Electronics - A useful refresher on the basics of circuit analysis before I tutor it in the upcoming school year for the University of Calgary [PASS program](https://www.ucalgary.ca/student-services/student-success/learning-support/pass "PASS Program Homepage")

## Next Steps:
The only things left to do are:
1. Using complex matrices implement a program to solve AC circuitry for both phase and magnitude
2. Rewrite the logic to use inbuilt matrices on a TI-84

---

The bottleneck is that implementing complex matrices on a TI-84 is much more [Difficult](https://www.youtube.com/watch?v=LVB3_ANZDGQ&ab_channel=MoVoltageMoPower "Example youtube video") than on a library built for it. So for any progress, the switch to using TI 84 matrices must be done first.
