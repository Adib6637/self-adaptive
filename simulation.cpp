#include <systemc>
#include "parameter.h"
#include "self_adaptive.h"
#include "managed_system.h"
#include "weather_forecast.h"

extern void load_manuever_data();
extern void load_sensor_data();
extern void load_model_coefficient();
extern int rejected_data_counter;
extern std::vector<std::tuple<double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double>> manuever_data;
extern std::vector<std::tuple<double, double, double, double, double>> sensor_data;

int sc_main(int argc, char* argv[]) {

    std::cout << "Starting simulation..." << std::endl;
    if(!MANAGED_SYSTEM_ON){

        std::cout << "MANAGED_SYSTEM_ON is false. use coeeficients dataset for model (no learning step)." << std::endl;
        std::cout << "loadding coefficients dataset..." << std::endl;
        load_model_coefficient();
        std::cout << "Coefficients dataset loaded successfully." << std::endl;

    }else{
        std::cout << "Loading maneuver and sensor data..." << std::endl;
        load_manuever_data();
        std::cout << "Loading sensor data..." << std::endl;
        load_sensor_data();
        std::cout << "Data loaded successfully." << std::endl;
    }

    sc_clock clk("clk", SIMULATION_CLK_TICK_NS, SC_NS);

    // Use sc_vector of signals instead of raw arrays
    sc_vector<sc_signal<double>> sig_managed_system_data("sig_managed_system_data", 20);
    sc_vector<sc_signal<double>> sig_weather_prediction("sig_weather_prediction", 20);
    sc_vector<sc_signal<double>> sig_power_consumption("sig_power_consumption", 20);
    sc_vector<sc_signal<double>> sig_operation_time("sig_operation_time", 20);

    // Instantiate modules
    Self_Adaptive self_adaptive("self_adaptive");
    self_adaptive.clk(clk);
    self_adaptive.managed_system_data(sig_managed_system_data);
    self_adaptive.weather_prediction(sig_weather_prediction);
    self_adaptive.power_consumption(sig_power_consumption);
    self_adaptive.operation_time(sig_operation_time);

    Weather_Forecast weather_forecast("weather_forecast");
    weather_forecast.clk(clk);
    weather_forecast.weather_forecast(sig_weather_prediction);

    Managed_System managed_system("managed_system");
    managed_system.clk(clk);
    managed_system.system_data(sig_managed_system_data);

    std::cout << "OPTIMIZER_ON: " << OPTIMIZER_ON << std::endl;
    std::cout << "CONSTRAINT_TUNER_ON: " << CONSTRAINT_TUNER_ON << std::endl;
    std::cout << "MODEL_LEARNER_ON: " << MODEL_LEARNER_ON << std::endl;

    std::cout << "Simulation setup complete." << std::endl;
    std::cout << "Starting simulation..." << std::endl;
    sc_start(SIMULATION_DURATION_NS, SC_NS);

    
    std::cout << "Simulation finished." << std::endl;

    if(MANAGED_SYSTEM_ON){
        //std::cout << "sensor_data size: " << sensor_data.size() << std::endl;
        //std::cout << "manuever_data size: " << manuever_data.size() << std::endl;
        //std::cout << "Rejected manuever_data count: " << rejected_data_counter << std::endl;
        //std::cout << "manuever_data used for simulation: " <<  self_adaptive.model_learning->counter - rejected_data_counter << std::endl;
    }else{
        
    }


    //sc_start(1000, SC_NS);
    return 0;
}

