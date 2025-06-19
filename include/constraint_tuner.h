#ifndef CONSTRAINT_REFINING_H
#define CONSTRAINT_REFINING_H

#include <systemc>
#include <vector>
#include <string>

using namespace sc_core;

SC_MODULE(Constraint_Tuner) {
    sc_in<bool> clk;
    sc_core::sc_vector<sc_in<double>> observed_data;
    sc_core::sc_vector<sc_out<double>> constraint_value;

    
    std::vector<double> speed_data; 
    std::vector<double> power_data; 
    int counter;

    void tune();

    SC_CTOR(Constraint_Tuner)
        : observed_data("observed_data", 20),
          constraint_value("constraint_value", 20),
          counter(0)
    {
        SC_METHOD(tune);
        sensitive << clk.pos();
    }
};

#endif
