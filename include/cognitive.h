#ifndef COGNITIVE_H
#define COGNITIVE_H

#include <systemc>
#include <vector>
#include <string>
#include "model_learning.h"
#include "monitor.h"
#include "optimizer.h"
// #include "validator.h"

using namespace sc_core;

SC_MODULE(Cognitive) {
    // main clk
    sc_in<bool> clk;

    // module interface
    sc_vector<sc_in<double>> fms_data;
    sc_vector<sc_in<double>> weather_prediction;
    sc_vector<sc_out<double>> power_consumption;
    sc_vector<sc_out<double>> operation_time;

    // internal signal
    sc_vector<sc_signal<double>> sig_observed_data;     // between fms interface and monitor
    sc_vector<sc_signal<double>> sig_model_parameter;   // between model learner and optimizer
    sc_vector<sc_signal<double>> sig_cfg;               // between optimizer and executor
    sc_vector<sc_signal<double>> sig_operation_time;    // between optimizer and operation time interface
    sc_vector<sc_signal<double>> sig_power_consumption; // between optimizer and power consumption interface

    // sub module
    Model_Learning* model_learning;
    Monitor* monitor;
    Optimizer* optimizer;
    // Validator* validator;

    void run();

    SC_CTOR(Cognitive)
        : fms_data("fms_data", 20),
          weather_prediction("weather_prediction", 20),
          power_consumption("power_consumption", 20),
          operation_time("operation_time", 20),
          sig_observed_data("sig_observed_data", 20),
          sig_cfg("sig_cfg", 20),
          sig_model_parameter("sig_model_parameter", 20),
          sig_operation_time("sig_operation_time", 20),
          sig_power_consumption("sig_power_consumption", 20)
    {

        // SUB MODULE: monitor
        monitor = new Monitor("monitor");
        monitor->clk(clk);
        for (int i = 0; i < 20; ++i) {
            monitor->fms_data[i](fms_data[i]);
            monitor->observed_data[i](sig_observed_data[i]);
        }

        // SUB MODULE: model learning
        model_learning = new Model_Learning("model_learning");
        model_learning->clk(clk);
        for (int i = 0; i < 20; ++i) {
            model_learning->observed_data[i](sig_observed_data[i]);
            model_learning->model_parameter[i](sig_model_parameter[i]);
        }

        // SUB MODULE: optimizer
        optimizer = new Optimizer("optimizer");
        optimizer->clk(clk);
        for (int i = 0; i < 20; ++i) {
            optimizer->model_parameter[i](sig_model_parameter[i]);
            optimizer->weather_prediction[i](weather_prediction[i]);
            optimizer->cfg[i](sig_cfg[i]);
            optimizer->power_consumption[i](sig_power_consumption[i]);
            optimizer->operation_time[i](sig_operation_time[i]);
            optimizer->observed_data[i](sig_observed_data[i]);
        }

        // SUB MODULE: executor
        // validator = new Validator("validator");
        // validator->clk(clk);
        // for (int i = 0; i < 20; ++i) {
        //     validator->cfg[i](sig_cfg[i]);
        // }

        SC_METHOD(run);
        sensitive << clk.pos();
    }

    ~Cognitive() {
        delete model_learning;
        delete monitor;
        delete optimizer;
        //delete validator;
    }
};

#endif
