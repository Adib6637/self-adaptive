#ifndef FMS_H
#define FMS_H

#include <systemc>
#include <vector>
#include <string>

using namespace sc_core;

SC_MODULE(FMS) {
    sc_in<bool> clk;
    sc_core::sc_vector<sc_out<double>> system_data;

    void simulate();
    int counter;

    SC_CTOR(FMS)
        : system_data("system_data", 20),
        counter(0)
    {
        SC_METHOD(simulate);
        sensitive << clk.pos();
    }
};

#endif
