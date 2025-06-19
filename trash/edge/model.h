#ifndef EDGE_MODEL_H
#define EDGE_MODEL_H

#include <systemc>
#include <string>
using namespace sc_core;



SC_MODULE(Model) {
    
    sc_in<bool> clk;
    sc_out<double> power_consumption;
    sc_out<double> operation_time;
    sc_in<double> parameter[10];

    void inference();

    SC_CTOR(Model) {
        SC_METHOD(inference);
        sensitive << clk.pos();
    }
};

#endif



