#ifndef MANAGED_SYSTEM_H
#define MANAGED_SYSTEM_H

#include <systemc>
#include <vector>
#include <string>

using namespace sc_core;

SC_MODULE(Managed_System) {
    sc_in<bool> clk;
    sc_core::sc_vector<sc_out<double>> system_data;

    void simulate();
    int counter;

    SC_CTOR(Managed_System)
        : system_data("system_data", 20),
        counter(0)
    {
        SC_METHOD(simulate);
        sensitive << clk.pos();
    }
};

#endif
