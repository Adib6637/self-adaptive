#include "model_learning.h"
#include "parameter.h"
#include <cmath>
#include <fstream>
#include <ctime>
#include <chrono>

// function prototype
void log_loss_to_csv(double maneuver_loss, double sensor_loss, double data_number);
void log_coefficient_actuator( const Eigen::VectorXd& x, double data_number);
void log_coefficient_sensor( const Eigen::VectorXd& x, double data_number);
void log_A_B_C( double A, double B, double C, double data_number);
void load_model_coefficient();
Eigen::VectorXd compute_gradient_actuator(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& d,
    double data_number);

// offline coefficient data: to test without monitor sub module
std::vector<std::tuple<double, double, double, double, double, double, double, double, double, double>> coefficient_data;
std::string model_coefficient_data_file = "./dataset/model_parameter_set/model_coefficient_test_set.csv";

// learning process
void Model_Learning::learning() {
    // condition to proceed oterwise skip
    if (!MODEL_LEARNER_ON){
        for (int i = 0; i < coeff_actuator_size; ++i){
            model_parameter[i].write(coeff_actuator(i)); // eta, delta, alpha, beta
        }    
        for (int i = 4; i < coeff_actuator_size + coeff_sensor_size; ++i){
            model_parameter[i].write(coeff_sensor(i-coeff_actuator_size)); // a, b, c
        }
    }
    if (observed_data[19].read() == 0 || counter == observed_data[19].read()) return; 
    counter = observed_data[19].read();
    if (!FMS_ON) {
        simulate_coefficient_data();
        return;
    }

    // ############################################ Data Preprocessing ##############################################################
    // Data Preprocessing
    double acceleration_z_normalized = observed_data[0].read();                 // Normalized acceleration z
    double mass_d_normalized = (DRONE_MASS - MASS_MIN) / (MASS_MAX - MASS_MIN); // Drone mass
    double mass_p_normalized = observed_data[1].read();                         // Payload mass
    double altitude_normalized = observed_data[2].read()-0.0917;                // Altitude
    double wind_speed_normalized = observed_data[3].read();                     // Wind speed
    double wind_angle = observed_data[4].read();                                // Wind angle
    double speed_normalized = observed_data[5].read();                          // Speed of drone
    double power_actuator_normalized = observed_data[6].read();                 // Power actuator
    double power_actuator= (
        observed_data[6].read()* (
            POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN)) + POWER_ACTUATOR_MIN;     // Adjust power actuator to the range of power actuator
    double fps_normalized = observed_data[7].read();                            // Normalized fps
    double pix_normalized = observed_data[8].read();                            // Normalized pixels 
    double pix_x_normalized = observed_data[10].read();                         // Normalized pixel x
    double pix_y_normalized = observed_data[11].read();                         // Normalized pixel y
    double power_sensor_normalized = observed_data[9].read();                   // Normalized power sensor

    // ########################################## Actuator Power Model Update #########################################################
    Eigen::VectorXd actuator_input(8);
    actuator_input(0) = acceleration_z_normalized;                              // Normalized linear acceleration z
    actuator_input(1) = mass_d_normalized;                                      // m_d
    actuator_input(2) = mass_p_normalized;                                      // m_p
    actuator_input(3) = altitude_normalized-0.0917;                             // h
    actuator_input(4) = (H_REF-ALTITUDE_MIN)/(ALTITUDE_MAX-ALTITUDE_MIN);       // h_ref
    actuator_input(5) = wind_speed_normalized;                                  // v_windf
    actuator_input(6) = wind_angle;                                             // theta_wind
    actuator_input(7) = speed_normalized;                                       // v_i
    double power_actuator_measured = power_actuator_normalized;                 // Ground truth power consumption

#ifdef LEARNING_TIMING_LOG
    auto learning_start = std::chrono::steady_clock::now();
#endif
    
    // compute gradient
    Eigen::VectorXd grad_actuator = compute_gradient_actuator(coeff_actuator, actuator_input, observed_data[19].read());
    // Compute loss
    double power_actuator_pred = predict_actuator_power(coeff_actuator, actuator_input, observed_data[19].read());
    // Gradient descent update
    Eigen::VectorXd update_actuator = lr_actuator * (power_actuator_pred - power_actuator_measured) * grad_actuator;
    // Apply learning rate decay
    lr_actuator *= decay_actuator;
    // Update coefficients
    coeff_actuator -= update_actuator;
    // Write updated parameters
    for (int i = 0; i < coeff_actuator_size; ++i) {
        model_parameter[i].write(coeff_actuator(i));
    }
    double loss_actuator = 0.5 * std::pow(power_actuator_pred - power_actuator_measured, 2);
    
    // ########################################## Sensor Power Model Update #########################################################
    // Prepare input
    Eigen::VectorXd sensor_input(2);
    sensor_input(0) = fps_normalized;                                           // Normalized fps
    sensor_input(1) = pix_normalized;                                           // Normalized pixels
    double power_sensor_measured = power_sensor_normalized;

    // Predict power with current coefficients
    double power_sensor_pred = predict_sensor_power(coeff_sensor, sensor_input, observed_data[19].read());
    // Compute gradient
    Eigen::VectorXd grad_sensor(4);
    grad_sensor(0) = sensor_input(0);                                           // ∂P/∂x0 = fps
    grad_sensor(1) = sensor_input(1);                                           // ∂P/∂x1 = pix
    grad_sensor(2) = 1.0;                                                       // ∂P/∂x2 = bias term
    grad_sensor(3) = 0.0;                                                       // reserved
    // Gradient descent update
    Eigen::VectorXd update_sensor = lr_sensor * (power_sensor_pred - power_sensor_measured) * grad_sensor;
    // Learning rate decay
    lr_sensor *= decay_sensor;
    // Update coefficients
    coeff_sensor -= update_sensor;
    // Write updated parameters 
    for (int i = 4; i < coeff_actuator_size + coeff_sensor_size; ++i) {
        model_parameter[i].write(coeff_sensor(i - coeff_actuator_size));        // a, b, c
    }
    double loss_sensor = 0.5 * std::pow(power_sensor_pred - power_sensor_measured, 2);

#ifdef LEARNING_TIMING_LOG
    auto learning_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> learning_duration = learning_end - learning_start;
    std::ofstream timefile("./log/log_learning_time.csv", std::ios::app);
    if (timefile.is_open()) {
        timefile << learning_duration.count() << "," << observed_data[19].read() << std::endl;
        timefile.close();
    }
#endif 
#ifdef LEARNING_PROGRESS_LOG
    log_coefficient_sensor(coeff_sensor, observed_data[19].read());
    log_coefficient_actuator(coeff_actuator, observed_data[19].read());
    log_loss_to_csv(loss_actuator, loss_sensor, observed_data[19].read()); // maneuver_loss, sensor_loss, data_number
#endif
}


// helper function
// - predict actuator power consumption
double Model_Learning::predict_actuator_power(const Eigen::VectorXd& x, const Eigen::VectorXd& d, double data_number) {
    // parameter preparation
    double g = d(0)*(
        LINEAR_ACCELERATION_Z_MAX-LINEAR_ACCELERATION_Z_MIN) + LINEAR_ACCELERATION_Z_MIN;   // Adjust g to the range of linear acceleration z
    double m_d = d(1) * (MASS_MAX - MASS_MIN) + MASS_MIN;                                   // Adjust m_d to the range of drone mass
    double m_p = d(2) * (MASS_MAX - MASS_MIN) + MASS_MIN;                                   // Adjust m_p to the range of payload mass
    double h = d(3) * (ALTITUDE_MAX - ALTITUDE_MIN) + ALTITUDE_MIN;                         // Adjust h to the range of altitude
    double h_ref = H_REF;                                                                   // Adjust h_ref to the range of altitude
    double v_wind = d(5) * (WIND_SPEED_MAX - WIND_SPEED_MIN) + WIND_SPEED_MIN;              // Adjust v_wind to the range of wind speed
    double theta_wind = d(6) * (WIND_ANGLE_MAX - WIND_ANGLE_MIN) + WIND_ANGLE_MIN;          // Adjust theta_wind to the range of wind angle
    double v_i = d(7) * (SPEED_MAX - SPEED_MIN) + SPEED_MIN;                                // Adjust v_i to the range of speed
    double mass_sum = m_d + m_p;
    double sin_th = std::sin(theta_wind);

    // fold constant
    double A = g*(mass_sum)*(v_wind*sin_th+v_i); 
    double B = std::pow((v_wind*sin_th+v_i),3); 
    double C = (h/h_ref)*(mass_sum); 
    
    // compute power
    double nominator = A + x(0)*B + x(1)*C + x(3); 
    double denominator = x(2); 
#ifdef LEARNING_PROGRESS_LOG
    log_A_B_C(A, B, C, data_number);
#endif
    return (((nominator / (denominator + 1e-8))) - POWER_ACTUATOR_MIN) / (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN);

}

// - predict sensor power consumption
double Model_Learning::predict_sensor_power(const Eigen::VectorXd& x, const Eigen::VectorXd& d, double data_number) {
    double fps = d(0);
    double pix = d(1);
    //double pred_power = x(0)*fps + x(1)*pix + x(2)*pix*fps + x(3)*std::pow(fps,2) + x(4)*std::pow(pix,2) + x(5);
    double pred_power = (x(0)*fps + x(1)*pix) + x(2);     //###########
    return pred_power;
}

// - log loss value
void log_loss_to_csv(double maneuver_loss, double sensor_loss, double data_number) {
    std::ofstream file("./log/log_loss.csv", std::ios::app); // Open in append mode
    if (file.is_open()) {
        file << maneuver_loss << "," << sensor_loss <<"," << data_number << std::endl;
        file.close();
    }
}

// - log actuator models coefficient
void log_coefficient_actuator( const Eigen::VectorXd& x, double data_number) {
    std::ofstream file("./log/log_coefficient_actuator.csv", std::ios::app); // Open in append mode
    if (file.is_open()) {
        file <<  x(0) << "," 
             << x(1) << "," 
             << x(2) << "," 
             << x(3) << "," 
             << data_number << std::endl;
        file.close();
    }
}

// - log sensor models coefficient
void log_coefficient_sensor( const Eigen::VectorXd& x, double data_number) {
    std::ofstream file("./log/log_coefficient_sensor.csv", std::ios::app); // Open in append mode
    if (file.is_open()) {
        file <<  x(0) << "," 
             << x(1) << "," 
             << x(2) << "," 
             << x(3) << "," 
             << data_number << std::endl;
        file.close();
    }
}

// - log sensor const folding result
void log_A_B_C( double A, double B, double C, double data_number) {
    std::ofstream file("./log/log_A_B_C.csv", std::ios::app); // Open in append mode
    if (file.is_open()) {
        file <<  A << "," 
             << B << "," 
             << C << "," 
             << data_number << std::endl;
        file.close();
    }
}

void load_model_coefficient() {
  
    //read the file
    std::ifstream file(model_coefficient_data_file);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << model_coefficient_data_file << std::endl;
        return;
    }

    // the variable
    std::string line;

    // the variable
    double d_a_0;
    double d_a_1;
    double d_a_2;   
    double d_a_3;
    double d_s_0;
    double d_s_1;
    double d_s_2;
    double d_s_3;
    double d_s_4;
    double d_s_5;

    // read every line
    std::getline(file, line); // Skip header

    // extract data on each line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        std::getline(ss, token, ',');
        double d_a_0 = std::stod(token);
        std::getline(ss, token, ',');
        double d_a_1 = std::stod(token);
        std::getline(ss, token, ',');
        double d_a_2 = std::stod(token);
        std::getline(ss, token, ',');
        double d_a_3 = std::stod(token);
        std::getline(ss, token, ',');
        double d_s_0 = std::stod(token);
        std::getline(ss, token, ',');
        double d_s_1 = std::stod(token);
        std::getline(ss, token, ',');
        double d_s_2 = std::stod(token);
        std::getline(ss, token, ',');
        double d_s_3 = std::stod(token);
        std::getline(ss, token, ',');
        double d_s_4 = std::stod(token);
        std::getline(ss, token, ',');
        double d_s_5 = std::stod(token);

        coefficient_data.emplace_back(d_a_0, d_a_1, d_a_2, d_a_3, d_s_0, d_s_1, d_s_2, d_s_3, d_s_4, d_s_5);
    }
    PRINT_LOG(18, "coefficient_data size",coefficient_data.size())

}

void Model_Learning::simulate_coefficient_data(){

    auto set = coefficient_data[counter];
    double d_a_0 = std::get<0>(set);
    double d_a_1 = std::get<1>(set);
    double d_a_2 = std::get<2>(set);
    double d_a_3 = std::get<3>(set);
    double d_s_0 = std::get<4>(set);
    double d_s_1 = std::get<5>(set);
    double d_s_2 = std::get<6>(set);
    double d_s_3 = std::get<7>(set);
    double d_s_4 = std::get<8>(set);
    double d_s_5 = std::get<9>(set);

    coeff_actuator << d_a_0, d_a_1, d_a_2, d_a_3;
    coeff_sensor << d_s_0, d_s_1, d_s_2, d_s_3, d_s_4, d_s_5;

    for (int i = 0; i < 4; ++i){
        model_parameter[i].write(coeff_actuator(i));
    }
    for (int i = 4; i < 10; ++i){
        model_parameter[i].write(coeff_sensor(i-4));
    }
}

Eigen::VectorXd compute_gradient_actuator(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& d,
    double data_number)
{
    // parameter preparation
    Eigen::VectorXd grad(4);
    double g = d(0)*(
        LINEAR_ACCELERATION_Z_MAX-LINEAR_ACCELERATION_Z_MIN) + LINEAR_ACCELERATION_Z_MIN;
    double m_d = d(1)*(MASS_MAX - MASS_MIN) + MASS_MIN;
    double m_p = d(2)*(MASS_MAX - MASS_MIN) + MASS_MIN;
    double h = d(3)*(ALTITUDE_MAX - ALTITUDE_MIN) + ALTITUDE_MIN;
    double h_ref = H_REF;
    double v_wind = d(5)*(WIND_SPEED_MAX - WIND_SPEED_MIN) + WIND_SPEED_MIN;
    double theta_wind = d(6)*(WIND_ANGLE_MAX - WIND_ANGLE_MIN) + WIND_ANGLE_MIN;
    double v_i = d(7)*(SPEED_MAX - SPEED_MIN) + SPEED_MIN;
    double mass_sum = m_d + m_p;
    double sin_th = std::sin(theta_wind);

    // fold constant
    double A = g * (mass_sum) * (v_wind * sin_th + v_i);
    double B = std::pow((v_wind * sin_th + v_i), 3);
    double C = (h / h_ref) * (mass_sum);

    // compute power
    double numerator = A + x(0)*B + x(1)*C + x(3);
    double denominator = x(2) + 1e-8;
    double scale = 1.0 / (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN);

    // // Analytical partial derivatives
    // grad(0) = (-numerator / (denominator * denominator)) * scale;   // ∂P/∂x0
    // grad(1) = (B / denominator) * scale;                            // ∂P/∂x1
    // grad(2) = (C / denominator) * scale;                            // ∂P/∂x2
    // grad(3) = (A / denominator) * scale;                            // ∂P/∂x3

    // Analytical partial derivatives
    grad(0) = (B / denominator) * scale;                            // ∂P/∂x0
    grad(1) = (C / denominator) * scale;                            // ∂P/∂x1
    grad(2) = (-numerator / (denominator * denominator)) * scale;   // ∂P/∂x2
    grad(3) = (1.0 / denominator) * scale;                          // ∂P/∂x3

    return grad;
}
     