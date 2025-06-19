#ifndef SELF_ADAPTIVE_H
#define SELF_ADAPTIVE_H

#include <systemc>
#include <vector>
#include <string>
#include "constraint_tuner.h"
#include "model_learning.h"
#include "monitor.h"
#include "optimizer.h"
#include "validator.h"

using namespace sc_core;

SC_MODULE(Self_Adaptive) {
    sc_in<bool> clk;
    sc_vector<sc_in<double>> managed_system_data;
    sc_vector<sc_in<double>> weather_prediction;

    sc_vector<sc_out<double>> power_consumption;
    sc_vector<sc_out<double>> operation_time;

    sc_vector<sc_signal<double>> sig_observed_data;
    sc_vector<sc_signal<double>> sig_new_cfg;
    sc_vector<sc_signal<double>> sig_cfg;
    sc_vector<sc_signal<double>> sig_model_parameter;
    sc_vector<sc_signal<double>> sig_constraints_value;
    sc_vector<sc_signal<double>> sig_operation_time;
    sc_vector<sc_signal<double>> sig_power_consumption;

    Constraint_Tuner* constraint_tuner;
    Model_Learning* model_learning;
    Monitor* monitor;
    Optimizer* optimizer;
    Validator* validator;

    void run();

    SC_CTOR(Self_Adaptive)
        : managed_system_data("managed_system_data", 20),
          weather_prediction("weather_prediction", 20),
          power_consumption("power_consumption", 20),
          operation_time("operation_time", 20),
          sig_observed_data("sig_observed_data", 20),
          sig_new_cfg("sig_new_cfg", 20),
          sig_cfg("sig_cfg", 20),
          sig_model_parameter("sig_model_parameter", 20),
          sig_constraints_value("sig_constraints_value", 20),
          sig_operation_time("sig_operation_time", 20),
          sig_power_consumption("sig_power_consumption", 20)
    {
        // Bind internal module instances
        constraint_tuner = new Constraint_Tuner("constraint_refining");
        constraint_tuner->clk(clk);
        for (int i = 0; i < 20; ++i) {
            constraint_tuner->observed_data[i](sig_observed_data[i]);
            constraint_tuner->constraint_value[i](sig_constraints_value[i]);
        }

        model_learning = new Model_Learning("model_learning");
        model_learning->clk(clk);
        for (int i = 0; i < 20; ++i) {
            model_learning->observed_data[i](sig_observed_data[i]);
            model_learning->model_parameter[i](sig_model_parameter[i]);
        }

        monitor = new Monitor("monitor");
        monitor->clk(clk);
        for (int i = 0; i < 20; ++i) {
            monitor->managed_system_data[i](managed_system_data[i]);
            monitor->new_cfg[i](sig_new_cfg[i]);
            monitor->observed_data[i](sig_observed_data[i]);
        }

        optimizer = new Optimizer("optimizer");
        optimizer->clk(clk);
        for (int i = 0; i < 20; ++i) {
            optimizer->model_parameter[i](sig_model_parameter[i]);
            optimizer->constraints_value[i](sig_constraints_value[i]);
            optimizer->weather_prediction[i](weather_prediction[i]);
            optimizer->cfg[i](sig_cfg[i]);
            optimizer->power_consumption[i](sig_power_consumption[i]);
            optimizer->operation_time[i](sig_operation_time[i]);
            optimizer->observed_data[i](sig_observed_data[i]);
        }

        validator = new Validator("validator");
        validator->clk(clk);
        for (int i = 0; i < 20; ++i) {
            validator->cfg[i](sig_cfg[i]);
            validator->new_cfg[i](sig_new_cfg[i]);
        }

        // Connect output signals to module output ports
        for (int i = 0; i < 20; ++i) {
            //power_consumption[i](optimizer->power_consumption[i]);//(sig_power_consumption[i]);
            //operation_time[i](optimizer->operation_time[i]);//(sig_operation_time[i]);
        }

        SC_METHOD(run);
        sensitive << clk.pos();
    }

    ~Self_Adaptive() {
        delete constraint_tuner;
        delete model_learning;
        delete monitor;
        delete optimizer;
        delete validator;
    }
};

#endif
