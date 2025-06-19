#ifndef MONITOR_H
#define MONITOR_H

#include <systemc>
#include <vector>
#include <string>

using namespace sc_core;

SC_MODULE(Monitor) {
    sc_in<bool> clk;
    sc_core::sc_vector<sc_in<double>> managed_system_data;
    sc_core::sc_vector<sc_in<double>> new_cfg;
    sc_core::sc_vector<sc_out<double>> observed_data;

    void monitor();

    SC_CTOR(Monitor)
        : managed_system_data("managed_system_data", 20),
          new_cfg("new_cfg", 20),
          observed_data("observed_data", 20)
    {
        SC_METHOD(monitor);
        sensitive << clk.pos();
    }
};

#endif
