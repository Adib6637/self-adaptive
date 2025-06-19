#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <systemc>
#include <vector>
#include <string>

using namespace sc_core;

SC_MODULE(Optimizer) {
    sc_in<bool> clk;

    sc_core::sc_vector<sc_in<double>> model_parameter;
    sc_core::sc_vector<sc_in<double>> constraints_value;
    sc_core::sc_vector<sc_in<double>> weather_prediction;
    sc_core::sc_vector<sc_in<double>> observed_data;

    sc_core::sc_vector<sc_out<double>> cfg;
    sc_core::sc_vector<sc_out<double>> power_consumption;
    sc_core::sc_vector<sc_out<double>> operation_time;

    double counter;

    void optimize();

    SC_CTOR(Optimizer)
        : model_parameter("model_parameter", 20),
          constraints_value("constraints_value", 20),
          weather_prediction("weather_prediction", 20),
          observed_data("observed_data", 20),
          cfg("cfg", 20),
          power_consumption("power_consumption", 20),
          operation_time("operation_time", 20),
          counter(-1)
    {
        SC_METHOD(optimize);
        sensitive << clk.pos();
    }
};
#endif
