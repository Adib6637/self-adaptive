#include "model_learning.h"
#include "parameter.h"
#include <cmath>
#include <fstream>
#include <ctime>
#include <chrono>

void log_loss_to_csv(double maneuver_loss, double sensor_loss, double data_number);
void log_coefficient_actuator( double eta, double delta, double alpha, double beta, double data_number);
void log_coefficient_sensor( const Eigen::VectorXd& x, double data_number);
void log_A_B_C( double A, double B, double C, double data_number);


std::vector<std::tuple<double, double, double, double, double, double, double, double, double, double>> coefficient_data;
void load_model_coefficient();
std::string model_coefficient_data_file = "../dataset/model_parameter_set/model_coefficient_test_set.csv";


void Model_Learning::learning() {
    if (!MODEL_LEARNER_ON || observed_data[19].read() == 0 || counter == observed_data[19].read()) return; 
    counter = observed_data[19].read();
    
    if (!MANAGED_SYSTEM_ON) {
        simulate_coefficient_data();
        return;
    }

    // observed data
    double mass_d_normalized = (observed_data[0].read() - MASS_MIN) / (MASS_MAX - MASS_MIN); // Drone mass
    double mass_p_normalized = (observed_data[1].read() - MASS_MIN) / (MASS_MAX - MASS_MIN); // Payload mass
    double altitude_normalized = (observed_data[2].read() - ALTITUDE_MIN) / (ALTITUDE_MAX - ALTITUDE_MIN); // Altitude
    double wind_speed_normalized = (observed_data[3].read() - WIND_SPEED_MIN) / (WIND_SPEED_MAX - WIND_SPEED_MIN); // Wind speed
    double wind_angle = (observed_data[4].read() - WIND_ANGLE_MIN) / (WIND_ANGLE_MAX - WIND_ANGLE_MIN); // Wind angle
    double speed_normalized = (observed_data[5].read() - SPEED_MIN) / (SPEED_MAX - SPEED_MIN); // Speed of drone
    double power_actuator_normalized = (observed_data[6].read() - POWER_ACTUATOR_MIN) / (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN); // Power actuator

    double fps_normalized = (observed_data[7].read() - FPS_MIN) / (FPS_MAX - FPS_MIN); // Normalized fps
    //double pix_normalized = (observed_data[8].read() - PIXELS_MIN) / (PIXELS_MAX - PIXELS_MIN); // Normalized pixels
    double pix_normalized = ((LOG_PIXEL(observed_data[8].read())-LOG_PIXEL(PIXELS_MIN))/(LOG_PIXEL(PIXELS_MAX)-LOG_PIXEL(PIXELS_MIN))); // Normalized pixels ##########
    double pix_x_normalized = (observed_data[10].read() - PIX_X_MIN) / (PIX_X_MAX - PIX_X_MIN); // Normalized pixel x
    double pix_y_normalized = (observed_data[11].read() - PIX_Y_MIN) / (PIX_Y_MAX - PIX_Y_MIN); // Normalized pixel y
    double power_sensor_normalized = (observed_data[9].read() - POWER_SENSOR_MIN) / (POWER_SENSOR_MAX - POWER_SENSOR_MIN); // Normalized power sensor


    // ############################################################################# Actuator Power Model Update #############################################################################

    Eigen::VectorXd d(8);
    d(0) = (GRAVITY-(LINEAR_ACCELERATION_Z_MAX*(-1)))/((LINEAR_ACCELERATION_Z_MIN*(-1))-(LINEAR_ACCELERATION_Z_MAX*(-1))); // Normalized gravity
    d(1) = mass_d_normalized; // m_d
    d(2) = mass_p_normalized; // m_p
    d(3) = altitude_normalized; // h
    d(4) = H_REF; // h_ref
    d(5) = wind_speed_normalized; // v_wind
    d(6) = wind_angle; // theta_wind
    d(7) = speed_normalized; // v_i
    double power_actuator_measured = power_actuator_normalized; // Ground truth power consumption

#ifdef LEARNING_TIMING_LOG
    auto learning_start = std::chrono::steady_clock::now();
#endif

    Eigen::Vector4d grad;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector4d x_plus = coeff_actuator;
        x_plus(i) += epsilon_actuator;
        double y1 = predict_actuator_power(x_plus, d, observed_data[19].read());

        Eigen::Vector4d x_minus = coeff_actuator;
        x_minus(i) -= epsilon_actuator;
        double y2 = predict_actuator_power(x_minus, d, observed_data[19].read());

        grad(i) = (y1 - y2) / (2 * epsilon_actuator);
    }
    double power_actuator_pred = predict_actuator_power(coeff_actuator, d, observed_data[19].read());
    double loss_actuator = 0.5 * std::pow(power_actuator_pred - power_actuator_measured, 2);

    if (std::isnan(loss_actuator)) return;

    Eigen::Vector4d update = lr_actuator * (power_actuator_pred - power_actuator_measured) * grad;
    lr_actuator *= decay_actuator;
    for (int i = 0; i < 4; ++i) {
        coeff_actuator(i) -= update(i);
    } 

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
    double m_d = d(1);
    double m_p = d(2);
    double h = d(3);
    double h_ref = d(4);
    double v_wind = d(5);
    double theta_wind = d(6);
    double v_i = d(7);
    double mass_sum = m_d + m_p;
    double cos_th = std::cos(theta_wind);
    double sin_th = std::sin(theta_wind);
    double A = g*(mass_sum)*(v_wind*sin_th+v_i);
    double B = std::pow((v_wind*sin_th+v_i),3);
    double C = h*(mass_sum);
    double nominator = A+x(2)*B + x(3)*C;
    double denominator = x(0);
#ifdef LEARNING_PROGRESS_LOG
    log_A_B_C(  A,  B,  C,  data_number);
#endif
    return (nominator / (denominator + 1e-8))+x(1);
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
    std::cout << "coefficient_data size: " << coefficient_data.size() << std::endl;

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
