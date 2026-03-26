#include "Vtb_ramulator_dpi.h"
#include "verilated.h"

vluint64_t main_time = 0;

double sc_time_stamp() { return main_time; }

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    Vtb_ramulator_dpi* top = new Vtb_ramulator_dpi;

    while (!Verilated::gotFinish()) {
        top->eval();
        main_time++;
    }

    delete top;
    return 0;
}