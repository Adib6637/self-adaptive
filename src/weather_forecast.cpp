#include "weather_forecast.h"
#include "parameter.h"
#include <random>
#include <fstream>

//sc_in<bool> clk;
//sc_out<double> weather_forecast[10];

std::string file_name_csv = "../log/log_wind_forecast.csv"; 


void Weather_Forecast::forecast() {

    if(!DYNAMIC_WEATHER){
        weather_forecast[0].write(4.0);      // v_wind
        weather_forecast[1].write(150.0);  // theta_wind
        return;
    }
    
    static std::default_random_engine generator(std::random_device{}());
    static std::uniform_real_distribution<double> v_wind_delta_dist(0.0, 0.13);      // always positive increment
    static std::uniform_real_distribution<double> theta_wind_delta_dist(0.0, 1.0);  // always positive increment
    static double prev_v_wind = 3.0;      // initial mean value
    static double prev_theta_wind = 150.0; // initial mean value

    double v_wind = prev_v_wind + v_wind_delta_dist(generator);
    double theta_wind = prev_theta_wind + theta_wind_delta_dist(generator);

    // Clamp values to defined parameter ranges
    if (v_wind < WIND_SPEED_MIN) v_wind = WIND_SPEED_MIN;
    if (v_wind > WIND_SPEED_MAX) v_wind = WIND_SPEED_MAX;
    if (theta_wind < WIND_ANGLE_MIN) theta_wind = WIND_ANGLE_MIN;
    if (theta_wind > WIND_ANGLE_MAX) theta_wind = WIND_ANGLE_MAX;

    prev_v_wind = v_wind;
    prev_theta_wind = theta_wind;

    weather_forecast[0].write(v_wind);      // v_wind
    weather_forecast[1].write(theta_wind);  // theta_wind

    // Record v_wind and theta_wind in CSV
    static bool header_written = false;
    std::ofstream csv(file_name_csv, std::ios::app);
    if (!header_written && csv.tellp() == 0) {
        csv << "v_wind,theta_wind\n";
        header_written = true;
    }
    csv << v_wind << "," << theta_wind << "\n";
    csv.close();
    //std::cout << "Weather forecast updated: v_wind = " << v_wind << ", theta_wind = " << theta_wind << std::endl;
}


