#ifndef MODEL_LEARNING_H
#define MODEL_LEARNING_H

#include <systemc>
#include <vector>
#include <deque>
#include <Eigen/Dense>
#include <memory>
#include "mlp.h"

using namespace sc_core;

SC_MODULE(Model_Learning) {
    sc_in<bool> clk;
    sc_core::sc_vector<sc_in<double>> observed_data;
    sc_core::sc_vector<sc_out<double>> model_parameter;  

    Eigen::VectorXd coeff_actuator; // global parameters: [eta, delta, alpha, beta] for manuever
    Eigen::VectorXd coeff_sensor;  // global parameters: [sigma, omega, epsilon] for camera

    // synchronization variables
    double counter;

    // Neural networks for power prediction
    std::unique_ptr<MLP> actuator_mlp;
    std::unique_ptr<MLP> sensor_mlp;

    // Momentum parameters
    Eigen::VectorXd prev_grad_actuator;
    Eigen::VectorXd prev_grad_sensor;
    const double momentum = 0.9;

    // Moving average parameters
    const int window_size = 5;
    std::deque<Eigen::VectorXd> actuator_coeffs_history;
    std::deque<Eigen::VectorXd> sensor_coeffs_history;

    // Adaptive learning rate parameters
    double prev_loss_actuator;
    double prev_loss_sensor;
    double learning_rate_multiplier;
    const double lr_increase = 1.02;  // More conservative increase
    const double lr_decrease = 0.98;  // More conservative decrease

    void learning();
    double predict_actuator_power(const Eigen::VectorXd& coeff_actuator, const Eigen::VectorXd& d, double data_number);
    double predict_sensor_power(const Eigen::VectorXd& x, const Eigen::VectorXd& d, double data_number);
    void simulate_coefficient_data();

    SC_CTOR(Model_Learning)
        : observed_data("observed_data", 20),
          model_parameter("model_parameter", 20),
          prev_grad_actuator(Eigen::VectorXd::Zero(5)),
          prev_grad_sensor(Eigen::VectorXd::Zero(6)),
          prev_loss_actuator(std::numeric_limits<double>::max()),
          prev_loss_sensor(std::numeric_limits<double>::max()),
          learning_rate_multiplier(1.0),
          counter(-1)
    {
        coeff_sensor.resize(6);
        coeff_actuator.resize(5);
        //coeff_sensor << 0.1,0.1,0.1,0.1,0.1,0.1; //##########
        //coeff_sensor << 0.928977,0.620271,0.0407869,0.00561747,0.0628375,-5.18385;
        coeff_sensor << 0.0021,0.9842,1.6427,-0.0716,0,0;
        
        //coeff_actuator(1.5,0.0,3.0,1.5), 
        //coeff_actuator(-0.0716979,0.111695,0.158571,-0.0555981),
        coeff_actuator << 1.0,0.0,1.0,-1.386937e-04 , 1.0, // eta, 0, alpha, beta, A
        SC_METHOD(learning);
        sensitive << clk.pos();
        dont_initialize();
    }
};

#endif

// cfg for documentation
//coeff_actuator(0.329957,-0.45514,0.0968531,0.00146015),
//coeff_sensor << 0.1,0.1,0.1,0.1,0.1,0.1;



// last value for all component working
// coeff_actuator(0.316843,0.548003,0.0973508,0.00219821)
// coeff_sensor << 0.928977,0.620271,0.0407869,0.00561747,0.0628375,-5.18385;

/*
          coeff_actuator(0.3,0.1,0.1,0.002),
          epsilon_actuator(1e-4), 
          lr_actuator(1e-2),
          decay_actuator(0.98),
          epsilon_sensor(1e-4),
          lr_sensor(1e-2),
          decay_sensor(0.9999),
          counter(-1)
    {
        coeff_sensor.resize(6);
        //coeff_sensor << 0.1,0.1,0.1,0.1,0.1,0.1; //##########
        coeff_sensor << 1.3, 8.0, 0.0, 0, 0, 0;
        SC_METHOD(learning);
*/

/*

pa_eta: 0.318200000000000,
pa_delta: 0.289300000000000,
pa_alpha: 0.097300000000000,
pa_beta: 0.002100000000000,
ps_a: 0.984200000000000,
ps_b: 1.642700000000000,
ps_c: -0.071600000000000

*/