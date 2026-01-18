#ifndef MONITOR_H
#define MONITOR_H

#include <systemc>
#include <vector>
#include <string>

using namespace sc_core;

SC_MODULE(Monitor) {
    // main clk
    sc_in<bool> clk;

    // module interface
    sc_core::sc_vector<sc_in<double>> fms_data;
    sc_core::sc_vector<sc_out<double>> observed_data;

    void monitor();

    SC_CTOR(Monitor)
        : fms_data("fms_data", 20),
          observed_data("observed_data", 20)
    {
        SC_METHOD(monitor);
        sensitive << clk.pos();
    }
};

#endif
