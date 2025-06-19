#ifndef MODEL_LEARNING_H
#define MODEL_LEARNING_H

#include <systemc>
#include <vector>
#include <Eigen/Dense>

using namespace sc_core;

SC_MODULE(Model_Learning) {
    sc_in<bool> clk;
    sc_core::sc_vector<sc_in<double>> observed_data;
    sc_core::sc_vector<sc_out<double>> model_parameter;  

    Eigen::Vector4d coeff_actuator; // global parameters: [eta, delta, alpha, beta] for manuever
    Eigen::VectorXd coeff_sensor;  // global parameters: [sigma, omega, epsilon] for camera  

    // Gradient descent step actuator
    double epsilon_actuator;
    double lr_actuator;
    double decay_actuator;

    // Gradient descent step c
    double epsilon_sensor;
    double lr_sensor;
    double decay_sensor;

    // synchronization variables
    double counter;

    void learning();
    double predict_actuator_power(const Eigen::Vector4d& coeff_actuator, const Eigen::VectorXd& d, double data_number);
    double predict_sensor_power(const Eigen::VectorXd& x, const Eigen::VectorXd& d, double data_number);
    void simulate_coefficient_data();

    SC_CTOR(Model_Learning)
        : observed_data("observed_data", 20),
          model_parameter("model_parameter", 20),
          //coeff_actuator(0.329957,-0.45514,0.0968531,0.00146015), // initial guess [eta, delta, alpha, beta]    0.316843,0.548003,0.0973508,0.00219821 ##########
          coeff_actuator(0.316843,0.548003,0.0973508,0.00219821),
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
        coeff_sensor << 0.928977,0.620271,0.0407869,0.00561747,0.0628375,-5.18385;
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


