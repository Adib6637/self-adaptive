#ifndef MODEL_LEARNING_H
#define MODEL_LEARNING_H

#include <systemc>
#include <vector>
#include <deque>
#include <Eigen/Dense>
#include "parameter.h"

using namespace sc_core;

// interface size configuration
const int  coeff_sensor_size = 4;
const int  coeff_actuator_size = 4;

SC_MODULE(Model_Learning) {
    // main clk
    sc_in<bool> clk;

    // module interface
    sc_core::sc_vector<sc_in<double>> observed_data;
    sc_core::sc_vector<sc_out<double>> model_parameter;  

    // parameter holder
    // - coefficient 
    Eigen::VectorXd coeff_actuator; // global parameters: [eta, delta, alpha, beta] for manuever
    Eigen::VectorXd coeff_sensor;  // global parameters: [sigma, omega, epsilon] for camera
    // - Gradient descent step actuator
    double epsilon_actuator;
    double lr_actuator;
    double decay_actuator;
    // - Gradient descent step c
    double epsilon_sensor;
    double lr_sensor;
    double decay_sensor;
    // - synchronization variables
    double counter;

    void learning();
    double predict_actuator_power(const Eigen::VectorXd& coeff_actuator, const Eigen::VectorXd& d, double data_number);
    double predict_sensor_power(const Eigen::VectorXd& x, const Eigen::VectorXd& d, double data_number);
    void simulate_coefficient_data();

    SC_CTOR(Model_Learning)
        : observed_data("observed_data", 20),
          model_parameter("model_parameter", 20),
          counter(-1),
          epsilon_actuator(EPSILON_ACTUATOR_INIT), 
          lr_actuator(LR_ACTUATOR_INIT),
          decay_actuator(DECAY_ACTUATOR_INIT),
          epsilon_sensor(EPSILON_SENSOR_INIT),
          lr_sensor(LR_SENSOR_INIT),
          decay_sensor(DECAY_SENSOR_INIT)
    {
        coeff_sensor.resize(coeff_sensor_size);
        coeff_actuator.resize(coeff_actuator_size);
        coeff_sensor << (double)COEFFICIENT_SENSOR_A_INIT, (double)COEFFICIENT_SENSOR_B_INIT, (double)COEFFICIENT_SENSOR_C_INIT, 0.0; // a, b, c, d
        coeff_actuator << (double)COEFFICIENT_ACTUATOR_ETA_INIT, (double)COEFFICIENT_ACTUATOR_DELTA_INIT,(double)COEFFICIENT_ACTUATOR_ALPHA_INIT, (double)COEFFICIENT_ACTUATOR_BETA_INIT; // eta, delta, alpha, beta
        SC_METHOD(learning);
        sensitive << clk.pos();
        dont_initialize();
    }
};
#endif
