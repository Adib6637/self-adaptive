#include "managed_system.h"
#include "parameter.h"
#include <cstdlib>  
#include <ctime>    
#include <iostream>
#include <tuple>
#include <sstream>
#include <string>
#include <iomanip>
#include <vector>

std::vector<std::tuple<double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double>> manuever_data;
std::vector<std::tuple<double, double, double, double, double>> sensor_data;

extern double quaternionToYaw360(double x, double y, double z, double w);
extern double computeResultantSpeed(double vx, double vy, double vz);
int rejected_data_counter = 0;
void load_manuever_data();
void load_sensor_data();
bool hasExcessivePrecision(double value, int maxPrecision);

std::string manuever_data_file = "../dataset/flights.csv";
std::string sensor_data_file = "../dataset/camera.csv";

//sc_out<double> system_data[10]; //drone mass, payload mass, altitute, wind speed, wind angle, speed of drone,power actuator, umber of pixel, fps, power sensor
// drone -mass 3680

bool data_check(int counter);


void Managed_System::simulate(){
    if (!MANAGED_SYSTEM_ON){
        system_data[19].write(counter); // counter for the simulation
        counter+=1;
        return;
    }
    
    //std::cout << "counter " << counter <<std::endl; 
    if (!data_check(counter)) {
        //std::cout << "Data check failed for counter: " << counter << std::endl;
        rejected_data_counter++;
        counter+=1;
        return; 
    }
    
    auto data = manuever_data[counter];
    //std::cout << "counter: " << counter << std::endl;

    // actuator related
    double wind_speed = WIND_SPEED(data);
    double wind_angle = WIND_ANGLE(data);
    double battery_voltage = BATTERY_VOLTAGE(data);
    double battery_current = BATTERY_CURRENT(data);
    double position_x = POSITION_X(data);   
    double position_y = POSITION_Y(data);
    double position_z = POSITION_Z(data);
    double orientation_x = ORIENTATION_X(data);
    double orientation_y = ORIENTATION_Y(data);
    double orientation_z = ORIENTATION_Z(data);
    double orientation_w = ORIENTATION_W(data);
    double velocity_x = VELOCITY_X(data);
    double velocity_y = VELOCITY_Y(data);
    double velocity_z = VELOCITY_Z(data);
    double angular_x = ANGULAR_X(data);
    double angular_y = ANGULAR_Y(data);
    double angular_z = ANGULAR_Z(data);
    double linear_acceleration_x = LINEAR_ACCELERATION_X(data);
    double linear_acceleration_y = LINEAR_ACCELERATION_Y(data);
    double linear_acceleration_z = LINEAR_ACCELERATION_Z(data);
    double speed = SPEED(data);
    double payload = PAYLOAD(data);
    double altitude = ALTITUDE(data);   

    double drone_mass = DRONE_MASS;
    double total_mass = drone_mass + payload; 
    double power_actuator = battery_voltage * battery_current; // Power in watts
    double drone_angle_rad = quaternionToYaw360(orientation_x, orientation_y, orientation_z, orientation_w) * M_PI / 180.0; // Convert to radians
    double wind_angle_rad = wind_angle * M_PI / 180.0; // Convert to radians
    double wind_angle_relative = wind_angle_rad - drone_angle_rad; // Relative wind angle in radians
    wind_angle_relative -= (wind_angle_relative > M_PI) ? 2 * M_PI : 0; // Normalize to [-π, π]

    // normalize values
    double wind_speed_normalized = wind_speed / WIND_SPEED_MAX; // Normalize wind speed
    double battery_voltage_normalized = (battery_voltage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN); // Normalize battery voltage
    double battery_current_normalized = (battery_current - BATTERY_CURRENT_MIN) / (BATTERY_CURRENT_MAX - BATTERY_CURRENT_MIN); // Normalize battery current
    //double position_x_normalized = (position_x - POSITION_X_MIN) / (POSITION_X_MAX - POSITION_X_MIN); // Normalize position x
    //double position_y_normalized = (position_y - POSITION_Y_MIN) / (POSITION_Y_MAX - POSITION_Y_MIN); // Normalize position y
    //double position_z_normalized = (position_z - POSITION_Z_MIN) / (POSITION_Z_MAX - POSITION_Z_MIN); // Normalize position z
    //double orientation_x_normalized = (orientation_x - ORIENTATION_X_MIN) / (ORIENTATION_X_MAX - ORIENTATION_X_MIN); // Normalize orientation x
    //double orientation_y_normalized = (orientation_y - ORIENTATION_Y_MIN) / (ORIENTATION_Y_MAX - ORIENTATION_Y_MIN); // Normalize orientation y
    //double orientation_z_normalized = (orientation_z - ORIENTATION_Z_MIN) / (ORIENTATION_Z_MAX - ORIENTATION_Z_MIN); // Normalize orientation z
    //double orientation_w_normalized = (orientation_w - ORIENTATION_W_MIN) / (ORIENTATION_W_MAX - ORIENTATION_W_MIN); // Normalize orientation w
    //double velocity_x_normalized = (velocity_x - VELOCITY_X_MIN) / (VELOCITY_X_MAX - VELOCITY_X_MIN); // Normalize velocity x
    //double velocity_y_normalized = (velocity_y - VELOCITY_Y_MIN) / (VELOCITY_Y_MAX - VELOCITY_Y_MIN); // Normalize velocity y
    //double velocity_z_normalized = (velocity_z - VELOCITY_Z_MIN) / (VELOCITY_Z_MAX - VELOCITY_Z_MIN); // Normalize velocity z
    //double angular_x_normalized = (angular_x - ANGULAR_X_MIN) / (ANGULAR_X_MAX - ANGULAR_X_MIN); // Normalize angular x
    //double angular_y_normalized = (angular_y - ANGULAR_Y_MIN) / (ANGULAR_Y_MAX - ANGULAR_Y_MIN); // Normalize angular y
    //double angular_z_normalized = (angular_z - ANGULAR_Z_MIN) / (ANGULAR_Z_MAX - ANGULAR_Z_MIN); // Normalize angular z
    //double linear_acceleration_x_normalized = (linear_acceleration_x - LINEAR_ACCELERATION_X_MIN) / (LINEAR_ACCELERATION_X_MAX - LINEAR_ACCELERATION_X_MIN); // Normalize linear acceleration x
    //double linear_acceleration_y_normalized = (linear_acceleration_y - LINEAR_ACCELERATION_Y_MIN) / (LINEAR_ACCELERATION_Y_MAX - LINEAR_ACCELERATION_Y_MIN); // Normalize linear acceleration y
    //double linear_acceleration_z_normalized = (linear_acceleration_z - LINEAR_ACCELERATION_Z_MIN) / (LINEAR_ACCELERATION_Z_MAX - LINEAR_ACCELERATION_Z_MIN); // Normalize linear acceleration z
    double speed_normalized = (speed - SPEED_MIN) / (SPEED_MAX - SPEED_MIN); // Normalize speed
    double payload_normalized = (payload - PAYLOAD_MIN) / (PAYLOAD_MAX - PAYLOAD_MIN); // Normalize payload
    double altitude_normalized = (altitude - ALTITUDE_MIN) / (ALTITUDE_MAX - ALTITUDE_MIN); // Normalize altitude

    double drone_mass_normalized = (drone_mass - MASS_MIN) / (MASS_MAX - MASS_MIN); // Normalize drone mass
    double total_mass_normalized = (total_mass - MASS_MIN) / (MASS_MAX - MASS_MIN); // Normalize total mass
    double power_actuator_normalized = (power_actuator - POWER_ACTUATOR_MIN) / (POWER_ACTUATOR_MAX - POWER_ACTUATOR_MIN); // Normalize actuator power

    // sensor related
    auto data_sensor = sensor_data[counter];
    double fps = FPS(data_sensor);
    double pix = PIXELS(data_sensor);
    double pix_x = PIX_X(data_sensor);
    double pix_y = PIX_Y(data_sensor);
    double power_sensor = POWER_SENSOR(data_sensor);

    // normalize sensor data
    double fps_normalized = (fps - FPS_MIN) / (FPS_MAX - FPS_MIN); // Normalize fps
    double pix_normalized = (pix - PIXELS_MIN) / (PIXELS_MAX - PIXELS_MIN); // Normalize pixels
    double pix_x_normalized = (pix_x - PIX_X_MIN) / (PIX_X_MAX - PIX_X_MIN); // Normalize pixel x
    double pix_y_normalized = (pix_y - PIX_Y_MIN) / (PIX_Y_MAX - PIX_Y_MIN); // Normalize pixel y
    double power_sensor_normalized = (power_sensor - POWER_SENSOR_MIN) / (POWER_SENSOR_MAX - POWER_SENSOR_MIN); // Normalize power sensor


    system_data[0].write(drone_mass_normalized); // drone mass
    system_data[1].write(payload_normalized); // payload mass
    system_data[2].write(altitude_normalized); // altitude
    system_data[3].write(wind_speed_normalized); // wind speed
    system_data[4].write(wind_angle_relative); // wind angle relative to drone
    system_data[5].write(speed_normalized); // speed of drone
    system_data[6].write(power_actuator_normalized); // power actuator
    system_data[7].write(pix_normalized); // number of pixels
    system_data[8].write(fps_normalized); // fps
    system_data[9].write(power_sensor_normalized); // power sensor
    system_data[10].write(pix_x_normalized); // pixel x
    system_data[11].write(pix_y_normalized); // pixel y
    system_data[19].write(counter); // counter for the simulation
    counter+=1;
}

bool  data_check(int counter) {
    if (counter > 800000) {
        std::cout << "Max data reached, stopping simulation." << std::endl;
        while(1){} // Stop the simulation
    }
    auto data = manuever_data[counter];
    // wind speed
    if (WIND_SPEED(data) < WIND_SPEED_MIN || WIND_SPEED(data) > WIND_SPEED_MAX) {
        //std::cout << "Wind speed out of range: " << WIND_SPEED(data) << std::endl;
        return false;
    }
    // wind angle
    if (WIND_ANGLE(data) < WIND_ANGLE_MIN || WIND_ANGLE(data) > WIND_ANGLE_MAX) {
        //std::cout << "Wind angle out of range: " << WIND_ANGLE(data) << std::endl;
        return false;
    }
    // battery voltage
    if (BATTERY_VOLTAGE(data) < BATTERY_VOLTAGE_MIN || BATTERY_VOLTAGE(data) > BATTERY_VOLTAGE_MAX) {   
        //std::cout << "Battery voltage out of range: " << BATTERY_VOLTAGE(data) << std::endl;
        return false;
    }
    // battery current
    if (BATTERY_CURRENT(data) < BATTERY_CURRENT_MIN || BATTERY_CURRENT(data) > BATTERY_CURRENT_MAX) {
        //std::cout << "Battery current out of range: " << BATTERY_CURRENT(data) << std::endl;
        return false;
    }
    // position x
    // if (POSITION_X(data) < POSITION_X_MIN || POSITION_X(data) > POSITION_X_MAX) {
    //     std::cout << "Position X out of range: " << POSITION_X(data) << std::endl;
    //     return false;        
    // }
    // position y
    // if (POSITION_Y(data) < POSITION_Y_MIN || POSITION_Y(data) > POSITION_Y_MAX) {
    //     std::cout << "Position Y out of range: " << POSITION_Y(data) << std::endl;
    //     return false;
    // }
    // position z
    // if (POSITION_Z(data) < POSITION_Z_MIN || POSITION_Z(data) > POSITION_Z_MAX) {
    //     std::cout << "Position Z out of range: " << POSITION_Z(data) << std::endl;
    //     return false;
    // }
    // orientation x
    if (ORIENTATION_X(data) < ORIENTATION_X_MIN || ORIENTATION_X(data) > ORIENTATION_X_MAX) {
        //std::cout << "Orientation X out of range: " << ORIENTATION_X(data) << std::endl;
        return false;
    }
    // orientation y
    if (ORIENTATION_Y(data) < ORIENTATION_Y_MIN || ORIENTATION_Y(data) > ORIENTATION_Y_MAX) {
        //std::cout << "Orientation Y out of range: " << ORIENTATION_Y(data) << std::endl;
        return false;
    }
    // orientation z
    if (ORIENTATION_Z(data) < ORIENTATION_Z_MIN || ORIENTATION_Z(data) > ORIENTATION_Z_MAX) {
        //std::cout << "Orientation Z out of range: " << ORIENTATION_Z(data) << std::endl;    
        return false;
    }
    // orientation w
    if (ORIENTATION_W(data) < ORIENTATION_W_MIN || ORIENTATION_W(data) > ORIENTATION_W_MAX) {
        //std::cout << "Orientation W out of range: " << ORIENTATION_W(data) << std::endl;
        return false;
    }
    // velocity x   
    // if (VELOCITY_X(data) < VELOCITY_X_MIN || VELOCITY_X(data) > VELOCITY_X_MAX) {
    //     std::cout << "Velocity X out of range: " << VELOCITY_X(data) << std::endl;
    //     return false;
    // }
    // velocity y
    // if (VELOCITY_Y(data) < VELOCITY_Y_MIN || VELOCITY_Y(data) > VELOCITY_Y_MAX) {
    //     std::cout << "Velocity Y out of range: " << VELOCITY_Y(data) << std::endl;
    //     return false;
    // }
    // velocity z
    // if (VELOCITY_Z(data) < VELOCITY_Z_MIN || VELOCITY_Z(data) > VELOCITY_Z_MAX) {
    //     std::cout << "Velocity Z out of range: " << VELOCITY_Z(data) << std::endl;
    //     return false;
    // }
    // angular x
    // if (ANGULAR_X(data) < ANGULAR_X_MIN || ANGULAR_X(data) > ANGULAR_X_MAX) {
    //     std::cout << "Angular X out of range: " << ANGULAR_X(data) << std::endl;
    //     return false;
    // }
    // angular y
    // if (ANGULAR_Y(data) < ANGULAR_Y_MIN || ANGULAR_Y(data) > ANGULAR_Y_MAX) {
    //     std::cout << "Angular Y out of range: " << ANGULAR_Y(data) << std::endl;
    //     return false;    
    // }
    // angular z
    // if (ANGULAR_Z(data) < ANGULAR_Z_MIN || ANGULAR_Z(data) > ANGULAR_Z_MAX) {
    //     std::cout << "Angular Z out of range: " << ANGULAR_Z(data) << std::endl;
    //     return false;
    // }
    // linear acceleration x
    // if (LINEAR_ACCELERATION_X(data) < LINEAR_ACCELERATION_X_MIN || LINEAR_ACCELERATION_X(data) > LINEAR_ACCELERATION_X_MAX) {
    //     std::cout << "Linear Acceleration X out of range: " << LINEAR_ACCELERATION_X(data) << std::endl; 
    //     return false;
    // }
    // linear acceleration y
    // if (LINEAR_ACCELERATION_Y(data) < LINEAR_ACCELERATION_Y_MIN || LINEAR_ACCELERATION_Y(data) > LINEAR_ACCELERATION_Y_MAX) {
    //     std::cout << "Linear Acceleration Y out of range: " << LINEAR_ACCELERATION_Y(data) << std::endl;
    //     return false;
    // }
    //linear acceleration z
    if (LINEAR_ACCELERATION_Z(data) < LINEAR_ACCELERATION_Z_MIN || LINEAR_ACCELERATION_Z(data) > LINEAR_ACCELERATION_Z_MAX) {
        //std::cout << "Linear Acceleration Z out of range: " << LINEAR_ACCELERATION_Z(data) << std::endl;
        return false;
    }
    // speed
    if (SPEED(data) < SPEED_MIN || SPEED(data) > SPEED_MAX) {
        //std::cout << "Speed out of range: " << SPEED(data) << std::endl;
        return false;
    }
    // payload
    if (PAYLOAD(data) < PAYLOAD_MIN || PAYLOAD(data) > PAYLOAD_MAX) {
        //std::cout << "Payload out of range: " << PAYLOAD(data) << std::endl;
        return false;
    }
    // altitude
    if (ALTITUDE(data) < ALTITUDE_MIN || ALTITUDE(data) > ALTITUDE_MAX) {
        //std::cout << "Altitude out of range: " << ALTITUDE(data) << std::endl;
        return false;
    }
    // flight category
    if (FLIGHT_CAT < FLIGHT_CAT_MIN - 1 || FLIGHT_CAT > FLIGHT_CAT_MAX + 1) {
        //std::cout << "Flight category out of range: " << std::get<0>(data) << std::endl;
        return false;
    }
    // power
    double power_actuator = BATTERY_VOLTAGE(data) * BATTERY_CURRENT(data);
    if (power_actuator < POWER_ACTUATOR_MIN || power_actuator > POWER_ACTUATOR_MAX) {
        //std::cout << "Power actuator out of range: " << power_actuator << std::endl;
        return false;
    }
    // Check for excessive precision
    if (1
        //&& hasExcessivePrecision(WIND_SPEED(data), 3),  
        //&& hasExcessivePrecision(WIND_ANGLE(data), 3),
        && hasExcessivePrecision(BATTERY_VOLTAGE(data), 9) // battery voltage
        && hasExcessivePrecision(BATTERY_CURRENT(data), 10) // battery current
        //&& hasExcessivePrecision(POSITION_X(data), 9) // position x
        //&& hasExcessivePrecision(POSITION_Y(data), 9) // position y
        //&& hasExcessivePrecision(POSITION_Z(data), 8) // position z
        && hasExcessivePrecision(ORIENTATION_X(data), 10) // orientation x
        && hasExcessivePrecision(ORIENTATION_Y(data), 10) // orientation y
        && hasExcessivePrecision(ORIENTATION_Z(data), 10) // orientation z
        //&& hasExcessivePrecision(ORIENTATION_W(data), 10) // orientation w
        //&& hasExcessivePrecision(VELOCITY_X(data), 10) // velocity x
        //&& hasExcessivePrecision(VELOCITY_Y(data), 10) // velocity y
        //&& hasExcessivePrecision(VELOCITY_Z(data), 10) // velocity z
        //&& hasExcessivePrecision(ANGULAR_X(data), 10) // angular x
        //&& hasExcessivePrecision(ANGULAR_Y(data), 10) // angular y
        //&& hasExcessivePrecision(ANGULAR_Z(data), 10) // angular z
        //&& hasExcessivePrecision(LINEAR_ACCELERATION_X(data), 10) // linear acceleration x
        //&& hasExcessivePrecision(LINEAR_ACCELERATION_Y(data), 10) // linear acceleration y
        //&& hasExcessivePrecision(LINEAR_ACCELERATION_Z(data), 10) // linear acceleration z
        && hasExcessivePrecision(SPEED(data), 10) // speed
        && hasExcessivePrecision(PAYLOAD(data), 10) // payload
        && hasExcessivePrecision(ALTITUDE(data), 10) // altitude
    ) {
        // pass
    } else {
        //std::cout << "Data has excessive precision." << std::endl;
        return false;   
    }
    
    // Check for invalid values
    if (std::isnan(WIND_SPEED(data)) || std::isnan(WIND_ANGLE(data)) || std::isnan(BATTERY_VOLTAGE(data)) || std::isnan(BATTERY_CURRENT(data)) ||
        std::isnan(POSITION_X(data)) || std::isnan(POSITION_Y(data)) || std::isnan(POSITION_Z(data)) ||
        std::isnan(ORIENTATION_X(data)) || std::isnan(ORIENTATION_Y(data)) || std::isnan(ORIENTATION_Z(data)) || std::isnan(ORIENTATION_W(data)) ||
        std::isnan(VELOCITY_X(data)) || std::isnan(VELOCITY_Y(data)) || std::isnan(VELOCITY_Z(data)) ||
        std::isnan(ANGULAR_X(data)) || std::isnan(ANGULAR_Y(data)) || std::isnan(ANGULAR_Z(data)) ||
        std::isnan(LINEAR_ACCELERATION_X(data)) || std::isnan(LINEAR_ACCELERATION_Y(data)) || std::isnan(LINEAR_ACCELERATION_Z(data)) ||
        std::isnan(SPEED(data)) || std::isnan(PAYLOAD(data)) || std::isnan(ALTITUDE(data))) {
        //std::cout << "Data contains NaN values." << std::endl;
        return false;
    }
    return true;

}

void load_sensor_data() {
  
    //read the file
    std::ifstream file(sensor_data_file);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << sensor_data_file << std::endl;
        return;
    }

    // the variable
    std::string line;

    // the variable
    double fps_val ;
    double power_val;
    double pix_val;


    // read every line
    std::getline(file, line); // Skip header

    // extract data on each line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        std::getline(ss, token, ',');
        double fps_val = std::stod(token);
        std::getline(ss, token, ',');
        double power_val = std::stod(token);
        std::getline(ss, token, ',');
        double pix_val = std::stod(token);
        std::getline(ss, token, ',');
        double res_x_val = std::stod(token);
        std::getline(ss, token, ',');
        double res_y_val = std::stod(token);

        sensor_data.emplace_back(fps_val, power_val, pix_val, res_x_val,res_y_val);
    }
    std::cout << "sensor_data size: " << sensor_data.size() << std::endl;
}

void load_manuever_data() {
  
    //read the file
    std::ifstream file(manuever_data_file);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << manuever_data_file << std::endl;
        return;
    }

    // the variable
    std::string line;
    double flight_val;
    double time_val;
    double wind_speed_val;
    double wind_angle_val;
    double battery_voltage_val;
    double battery_current_val;
    double position_x_val;
    double position_y_val;
    double position_z_val;
    double orientation_x_val;
    double orientation_y_val;
    double orientation_z_val;
    double orientation_w_val;
    double velocity_x_val;
    double velocity_y_val;
    double velocity_z_val;
    double angular_x_val;
    double angular_y_val;
    double angular_z_val;
    double linear_acceleration_x_val;
    double linear_acceleration_y_val;
    double linear_acceleration_z_val;
    double speed_val;
    double payload_val;
    double altitude_val;

    // read every line
    std::getline(file, line); // Skip header

    // extract data on each line
    // flight	time	wind_speed	wind_angle	battery_voltage	battery_current	position_x	position_y	position_z	orientation_x	orientation_y	orientation_z	orientation_w	velocity_x	velocity_y	velocity_z	angular_x	angular_y	angular_z	linear_acceleration_x	linear_acceleration_y	linear_acceleration_z	speed	payload	altitude
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        std::getline(ss, token, ',');
        double flight_val = std::stod(token);
        std::getline(ss, token, ',');
        double time_val = std::stod(token);
        std::getline(ss, token, ',');
        double wind_speed_val = std::stod(token);
        std::getline(ss, token, ',');
        double wind_angle_val = std::stod(token);
        std::getline(ss, token, ',');
        double battery_voltage_val = std::stod(token);
        std::getline(ss, token, ',');
        double battery_current_val = std::stod(token);
        std::getline(ss, token, ',');
        double position_x_val = std::stod(token);
        std::getline(ss, token, ',');
        double position_y_val = std::stod(token);
        std::getline(ss, token, ',');
        double position_z_val = std::stod(token);
        std::getline(ss, token, ',');
        double orientation_x_val = std::stod(token);
        std::getline(ss, token, ',');
        double orientation_y_val = std::stod(token);
        std::getline(ss, token, ',');
        double orientation_z_val = std::stod(token);
        std::getline(ss, token, ',');
        double orientation_w_val = std::stod(token);
        std::getline(ss, token, ',');
        double velocity_x_val = std::stod(token);
        std::getline(ss, token, ',');
        double velocity_y_val = std::stod(token);
        std::getline(ss, token, ',');
        double velocity_z_val = std::stod(token);
        std::getline(ss, token, ',');
        double angular_x_val = std::stod(token);
        std::getline(ss, token, ',');
        double angular_y_val = std::stod(token);
        std::getline(ss, token, ',');
        double angular_z_val = std::stod(token);
        std::getline(ss, token, ',');
        double linear_acceleration_x_val = std::stod(token);
        std::getline(ss, token, ',');
        double linear_acceleration_y_val = std::stod(token);
        std::getline(ss, token, ',');
        double linear_acceleration_z_val = std::stod(token);
        std::getline(ss, token, ',');
        double speed_val = std::stod(token);
        std::getline(ss, token, ',');
        double payload_val = std::stod(token);
        std::getline(ss, token, ',');
        double altitude_val = std::stod(token);

        manuever_data.emplace_back(
            flight_val,
            time_val,
            wind_speed_val,
            wind_angle_val,
            battery_voltage_val,
            battery_current_val,
            position_x_val,
            position_y_val,
            position_z_val,
            orientation_x_val,
            orientation_y_val,
            orientation_z_val,
            orientation_w_val,
            velocity_x_val,
            velocity_y_val,
            velocity_z_val,
            angular_x_val,
            angular_y_val,
            angular_z_val,
            linear_acceleration_x_val,
            linear_acceleration_y_val,
            linear_acceleration_z_val,
            speed_val,
            payload_val,
            altitude_val
        );
    }
    std::cout << "manuever_data size: " << manuever_data.size() << std::endl;
}

bool hasExcessivePrecision(double value, int maxPrecision) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(maxPrecision + 1) << value;
        std::string strValue = oss.str();
        // Find the decimal point
        size_t decimalPos = strValue.find('.');
        if (decimalPos != std::string::npos) {
            std::string decimals = strValue.substr(decimalPos + 1);
            return decimals.length() > maxPrecision;
        }
        return false;
}