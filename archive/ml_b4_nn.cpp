#include "model_learning.h"
#include "parameter.h"
#include <cmath>
#include <fstream>
#include <ctime>
#include <chrono>
#include <iomanip>

void log_loss_to_csv(double maneuver_loss, double sensor_loss, double data_number);
void log_coefficient_actuator( double eta, double delta, double alpha, double beta, double data_number);
void log_coefficient_sensor( const Eigen::VectorXd& x, double data_number);
void log_A_B_C( double A, double B, double C, double data_number);

std::vector<std::tuple<double, double, double, double, double, double, double, double, double, double>> coefficient_data;
void load_model_coefficient();
std::string model_coefficient_data_file = "../dataset/model_parameter_set/model_coefficient_test_set.csv";

int mode =1; // use normalize var or not


void Model_Learning::learning() {
    //std::cout << "counter " << counter <<std::endl; 
    //std::cout << "counter " << observed_data[19].read() <<std::endl; 
    if (!MODEL_LEARNER_ON || observed_data[19].read() == 0 || counter == observed_data[19].read()) return; 
    counter = observed_data[19].read();
    
    if (!MANAGED_SYSTEM_ON) {
        simulate_coefficient_data();
        return;
    }

    // observed data
    double acceleration_z_normalized = observed_data[0].read(); // Normalized acceleration z
    double mass_d_normalized = (DRONE_MASS - MASS_MIN) / (MASS_MAX - MASS_MIN); // Drone mass
    double mass_p_normalized = observed_data[1].read(); // Payload mass
    double altitude_normalized = observed_data[2].read(); // Altitude
    double wind_speed_normalized = observed_data[3].read(); // Wind speed
    double wind_angle = observed_data[4].read(); // Wind angle
    double speed_normalized = observed_data[5].read(); // Speed of drone
    double power_actuator_normalized = observed_data[6].read(); // Power actuator
    double power_actuator= (observed_data[6].read()* (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN)) + POWER_ACTUATOR_MIN; // Adjust power actuator to the range of power actuator

    double fps_normalized = observed_data[7].read(); // Normalized fps
    //double pix_normalized = observed_data[8].read(); // Normalized pixels
    double pix_normalized = observed_data[8].read(); // Normalized pixels ##########
    double pix_x_normalized = observed_data[10].read(); // Normalized pixel x
    double pix_y_normalized = observed_data[11].read(); // Normalized pixel y
    double power_sensor_normalized = observed_data[9].read(); // Normalized power sensor

    // ############################################################################# Actuator Power Model Update #############################################################################


    Eigen::VectorXd d(8);
    d(0) = acceleration_z_normalized; // Normalized linear acceleration z
    d(1) = mass_d_normalized; // m_d
    d(2) = mass_p_normalized; // m_p
    d(3) = altitude_normalized; // h
    d(4) = (H_REF-ALTITUDE_MIN)/(ALTITUDE_MAX-ALTITUDE_MIN); // h_ref
    d(5) = wind_speed_normalized; // v_wind
    d(6) = wind_angle; // theta_wind
    d(7) = speed_normalized; // v_i
    double power_actuator_measured[] = {power_actuator_normalized, power_actuator}; // Ground truth power consumption

#ifdef LEARNING_TIMING_LOG
    auto learning_start = std::chrono::steady_clock::now();
#endif

    Eigen::Vector4d grad;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector4d x_plus = coeff_actuator;
        x_plus(i) += epsilon_actuator;
        double y1 = (predict_actuator_power(x_plus, d, observed_data[19].read()) - POWER_ACTUATOR_MIN) / (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN); // Adjust to normalized range

        Eigen::Vector4d x_minus = coeff_actuator;
        x_minus(i) -= epsilon_actuator;
        double y2 = (predict_actuator_power(x_minus, d, observed_data[19].read()) - POWER_ACTUATOR_MIN) / (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN); // Adjust to normalized range

        grad(i) = (y1 - y2) / (2 * epsilon_actuator);
    }
    double power_actuator_pred = (predict_actuator_power(coeff_actuator, d, observed_data[19].read()) - POWER_ACTUATOR_MIN) / (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN); // Adjust to normalized range
    double loss_actuator = 0.5 * std::pow(power_actuator_pred - power_actuator_measured[0], 2);

    if (std::isnan(loss_actuator)) return;

    Eigen::Vector4d update = lr_actuator * (power_actuator_pred - power_actuator_measured[0]) * grad;
    lr_actuator *= decay_actuator;
    for (int i = 0; i < 4; ++i) {
        coeff_actuator(i) -= update(i);
    } 

    std::cout << "error: " << power_actuator_pred - power_actuator_measured[0] << std::endl;
    std::cout << "power_actuator_pred: " << power_actuator_pred << std::endl;
    std::cout << "power_actuator_measured: " << power_actuator_measured[0] << std::endl;
    std::cout << "coeff_actuator: " << coeff_actuator.transpose() << std::endl;
    // ############################################################################# Sensor Power Model Update #############################################################################
    Eigen::VectorXd d_s(2);
    d_s(0) = fps_normalized; // Normalized fps
    d_s(1) = pix_normalized; // Normalized pixels

    Eigen::VectorXd grad_sensor(6);
    for (int i = 0; i < 6; ++i) {
        Eigen::VectorXd x_plus = coeff_sensor;
        x_plus(i) += epsilon_sensor;
        double y1 = predict_sensor_power(x_plus, d_s, observed_data[19].read());

        Eigen::VectorXd x_minus = coeff_sensor;
        x_minus(i) -= epsilon_sensor;
        double y2 = predict_sensor_power(x_minus, d_s, observed_data[19].read());

        grad_sensor(i) = (y1 - y2) / (2 * epsilon_sensor);
    }

    double power_sensor_measured = power_sensor_normalized;
    double power_sensor_pred = predict_sensor_power(coeff_sensor, d, observed_data[19].read());
    double loss_sensor = 0.5 * std::pow(power_sensor_pred - power_sensor_measured, 2);


    if (std::isnan(loss_sensor)) return;
    Eigen::VectorXd update_sensor_coefficient(6);
    update_sensor_coefficient = lr_sensor * (power_sensor_pred - power_sensor_measured) * grad_sensor;
    lr_sensor*=decay_sensor;
    for (int i = 0; i < 6; ++i) {
        coeff_sensor(i) -= update_sensor_coefficient(i);
    }
    

#ifdef LEARNING_TIMING_LOG
    auto learning_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> learning_duration = learning_end - learning_start;
    std::ofstream timefile("../log/log_learning_time.csv", std::ios::app);
    if (timefile.is_open()) {
        timefile << learning_duration.count() << "," << observed_data[19].read() << std::endl;
        timefile.close();
    }
#endif 

    // #################################################################################### Update Model ####################################################################################

    for (int i = 0; i < 4; ++i){
        model_parameter[i].write(coeff_actuator(i));
    }
    for (int i = 4; i < 10; ++i){
        model_parameter[i].write(coeff_sensor(i-4));
    }

#ifdef LEARNING_PROGRESS_LOG
    log_coefficient_sensor(coeff_sensor, observed_data[19].read());
    log_coefficient_actuator(coeff_actuator(0), coeff_actuator(1), coeff_actuator(2), coeff_actuator(3), observed_data[19].read());
    log_loss_to_csv(loss_actuator, loss_sensor, observed_data[19].read()); // maneuver_loss, sensor_loss, data_number
#endif
}

double Model_Learning::predict_actuator_power(const Eigen::Vector4d& x, const Eigen::VectorXd& d, double data_number) {
    double g = d(0);
    double g_ = d(0)*(LINEAR_ACCELERATION_Z_MAX-LINEAR_ACCELERATION_Z_MIN) + LINEAR_ACCELERATION_Z_MIN; // Adjust g to the range of linear acceleration z
    double m_d = d(1);
    double m_d_ = m_d * (MASS_MAX - MASS_MIN) + MASS_MIN; // Adjust m_d to the range of drone mass
    double m_p = d(2);
    double m_p_ = m_p * (MASS_MAX - MASS_MIN) + MASS_MIN; // Adjust m_p to the range of payload mass
    double h = d(3);
    double h_ = h * (ALTITUDE_MAX - ALTITUDE_MIN) + ALTITUDE_MIN; // Adjust h to the range of altitude
    double h_ref = d(4);
    double h_ref_ = H_REF; // Adjust h_ref to the range of altitude
    double v_wind = d(5);
    double v_wind_ = v_wind * (WIND_SPEED_MAX - WIND_SPEED_MIN) + WIND_SPEED_MIN; // Adjust v_wind to the range of wind speed
    double theta_wind = d(6);
    double theta_wind_ = theta_wind * (WIND_ANGLE_MAX - WIND_ANGLE_MIN) + WIND_ANGLE_MIN; // Adjust theta_wind to the range of wind angle
    double v_i = d(7);
    double v_i_ = v_i * (SPEED_MAX - SPEED_MIN) + SPEED_MIN; // Adjust v_i to the range of speed

    double mass_sum = m_d + m_p;
    double mass_sum_ = m_d_ + m_p_;
    double cos_th = std::cos(theta_wind);
    double cos_th_ = std::cos(theta_wind_);
    double sin_th = std::sin(theta_wind);
    double sin_th_ = std::sin(theta_wind_);

    double A = g*(mass_sum)*(v_wind*sin_th+v_i);
    double A_ = g_*(mass_sum_)*(v_wind_*sin_th_+v_i_); // Adjust A to the range of linear acceleration z
    double B = std::pow((v_wind*sin_th+v_i),3);
    double B_ = std::pow((v_wind_*sin_th_+v_i_),3); // Adjust B to the range of wind speed and speed
    double C = (h/h_ref)*(mass_sum);
    double C_ = (h_/h_ref_)*(mass_sum_); // Adjust C to the range of altitude and h_ref

    double nominator = A+x(2)*B + x(3)*C;
    double nominator_ = A_ + x(2)*B_ + x(3)*C_; // Adjust nominator to the range of linear acceleration z, wind speed, speed, altitude and h_ref
    double denominator = x(0);
    double denominator_ = x(0); // Adjust denominator to the range of drone mass
#ifdef LEARNING_PROGRESS_LOG
    log_A_B_C(  A,  B,  C,  data_number);
    //log_A_B_C(A_, B_, C_, data_number); // Log the adjusted values
#endif
    //return (nominator_ / (denominator_ + 1e-8)) + x(1); // Add a small value to avoid division by zero
    //std::cout << "(nominator_ / (denominator_ + 1e-8))+x(1): " << (nominator_ / (denominator_ + 1e-8))+x(1) << C << std::endl;
    double r[] = {(nominator / (denominator + 1e-8)), (nominator_ / (denominator_ + 1e-8))};
    return r[mode];//{(nominator / (denominator + 1e-8))+x(1), (nominator_ / (denominator_ + 1e-8))+x(1)}; // Add a small value to avoid division by zero

}

double Model_Learning::predict_sensor_power(const Eigen::VectorXd& x, const Eigen::VectorXd& d, double data_number) {
    double fps = d(0);
    double pix = d(1);
    //double pred_power = x(0)*fps + x(1)*pix + x(2)*pix*fps + x(3)*std::pow(fps,2) + x(4)*std::pow(pix,2) + x(5);
    double pred_power = (x(0)*fps + x(1)*pix) + x(2);     //###########
    return pred_power;
}

void log_loss_to_csv(double maneuver_loss, double sensor_loss, double data_number) {
    std::ofstream file("../log/log_loss.csv", std::ios::app); // Open in append mode
    if (file.is_open()) {
        file << maneuver_loss << "," << sensor_loss <<"," << data_number << std::endl;
        file.close();
    }
}

void log_coefficient_actuator( double eta, double delta, double alpha, double beta, double data_number) {
    std::ofstream file("../log/log_coefficient_actuator.csv", std::ios::app); // Open in append mode
    if (file.is_open()) {
        file <<  eta << "," 
             << delta << "," 
             << alpha << "," 
             << beta <<"," << data_number << std::endl;
        file.close();
    }
}

void log_coefficient_sensor( const Eigen::VectorXd& x, double data_number) {
    std::ofstream file("../log/log_coefficient_sensor.csv", std::ios::app); // Open in append mode
    if (file.is_open()) {
        file <<  x(0) << "," 
             << x(1) << "," 
             << x(2) << "," 
             << x(3) << "," 
             << x(4) << "," 
             << x(5) << "," 
             << data_number << std::endl;
        file.close();
    }
}

void log_A_B_C( double A, double B, double C, double data_number) {
    std::ofstream file("../log/log_A_B_C.csv", std::ios::app); // Open in append mode
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

//0.834243	33.0111	-6.32461	0.1	0.1	0.1
