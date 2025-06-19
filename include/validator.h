#ifndef VALIDATOR_H
#define VALIDATOR_H

#include <systemc>
#include <string>
#include <vector>

using namespace sc_core;

SC_MODULE(Validator) {
    sc_in<bool> clk;
    sc_core::sc_vector<sc_in<double>> cfg;
    sc_core::sc_vector<sc_out<double>> new_cfg;

    void validate();

    SC_CTOR(Validator)
        : cfg("cfg", 20),
          new_cfg("new_cfg", 20)
    {
        SC_METHOD(validate);
        sensitive << clk.pos();
    }
};

#endif
