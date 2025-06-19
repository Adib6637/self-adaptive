#include "UAV_sensor.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>

std::vector<std::tuple<int, double, double>> power_data;



void load_power_data(const std::string& filename) {
  

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }
    std::string line;
    int fps_val;
    double res_val, power_val;

        
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;

        std::getline(ss, token, ',');
        int fps_val = std::stoi(token);

        std::getline(ss, token, ',');
        double res_val = std::stod(token);

        std::getline(ss, token, ',');
        double power_val = std::stod(token);

    // std::cout << "Loaded: fps=" << fps_val << ", res=" << res_val << ", power=" << power_val << std::endl;

        power_data.emplace_back(fps_val, res_val, power_val);
    }

}

void Camera::power_consumption() {
    int current_fps = fps.read();
    double current_res = resolution.read();


    for (const auto& entry : power_data) {
        if (std::get<0>(entry) == current_fps && std::get<1>(entry) == current_res) {
            power.write(std::get<2>(entry));
            return;
        }
    }

    // Default or error value if no match found
    power.write(0.0);
}
