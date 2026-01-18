#include <systemc>
#include <iomanip>
#include "parameter.h"
#include "cognitive.h"
#include "weather_forecast.h"
#include "FMS.h"

// Declarations for external functions and variables
extern void load_manuever_data();
extern void load_sensor_data();
extern void load_model_coefficient();
extern void shuffle_data();
extern int rejected_data_counter;
extern std::vector<std::tuple<double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double, double>> manuever_data;
extern std::vector<std::tuple<double, double, double, double, double>> sensor_data;

// Utility function to convert array to string for logging
template <typename T, size_t N>
std::string arrayToString(const T (&arr)[N]);

// Main simulation entry point
int sc_main(int argc, char* argv[]) {

    // Log simulation setup parameters
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "start","simulation setup")
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "SIM_CASE", SIMULATION_CASE)
    PRINT_LOG(18, "SIM_SUB_CASE",SIMULATION_SUB_CASE)
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "OPTIMIZER_ON",((OPTIMIZER_ON)?"True":"False"))
    PRINT_LOG(18, "MODEL_LEARNER_ON",((MODEL_LEARNER_ON)?"True":"False"))
    PRINT_LOG(18, "DYNAMIC_WEATHER",((DYNAMIC_WEATHER)?"True":"False"))
    PRINT_LOG(18, "FMS_ON",((FMS_ON)?"True":"False"))
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "Field Area", FIELD_AREA)
    int drone_set_pix[] = DRONE_SET_PIX;
    int drone_set_pix_x[] = DRONE_SET_PIX_X;
    int drone_set_pix_y[] = DRONE_SET_PIX_Y;
    int drone_set_fps[] = DRONE_SET_FPS;
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "DRONE_SET_PIX",(arrayToString(drone_set_pix)))
    PRINT_LOG(18, "DRONE_SET_PIX_X",(arrayToString(drone_set_pix_x)))
    PRINT_LOG(18, "DRONE_SET_PIX_Y",(arrayToString(drone_set_pix_y)))
    PRINT_LOG(18, "DRONE_SET_FPS",(arrayToString(drone_set_fps)))
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "SIM_DURATION_NS", SIMULATION_DURATION_NS)
    PRINT_LOG(18, "SIM_CLK_TICK_NS", SIMULATION_CLK_TICK_NS)
    PRINT_LOG(18, "OPTIMIZER_GAP", OPTIMIZER_GAP)
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "NUMBER_DRONE_MAX", NUMBER_DRONE_MAX)
    PRINT_LOG(18, "DYNAMIC_DRONE", ((DYNAMIC_DRONE)?"True":"False"))
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "OPTIMIZE_INTERVAL", OPTIMIZE_INTERVAL)
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MAX", RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MAX)
    PRINT_LOG(18, "RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MIN", RESOLUTION_AREA_COVERED_PER_NUMBER_PIXEL_MIN)
    PRINT_LOG(18, "MAX_V_CAPTURING", MAX_V_CAPTURING)
    PRINT_LOG_NEW_LINE

    // Load datasets based on simulation mode
    if(!FMS_ON){
        PRINT_LOG(18, "load","coefficients dataset")
        load_model_coefficient();
        PRINT_LOG(18, "load","successfull")
    }else{
        PRINT_LOG(18, "load","manuever dataset")
        load_manuever_data();
        PRINT_LOG(18, "load","sensor dataset")
        load_sensor_data();
        PRINT_LOG(18, "load","successfull")
    }
    std::cout << std::endl;

    // Initialize SystemC clock
    sc_clock clk("clk", SIMULATION_CLK_TICK_NS, SC_NS);

    // Use sc_vector of signals instead of raw arrays
    sc_vector<sc_signal<double>> sig_fms_data("sig_fms_data", 20);
    sc_vector<sc_signal<double>> sig_weather_prediction("sig_weather_prediction", 20);
    sc_vector<sc_signal<double>> sig_power_consumption("sig_power_consumption", 20);
    sc_vector<sc_signal<double>> sig_operation_time("sig_operation_time", 20);

    // Instantiate modules
    //  MODULE: cognitive module
    Cognitive cognitive("cognitive");
    cognitive.clk(clk);
    cognitive.fms_data(sig_fms_data);
    cognitive.weather_prediction(sig_weather_prediction);
    cognitive.power_consumption(sig_power_consumption);
    cognitive.operation_time(sig_operation_time);

    // MODULE: Weather_Forecast
    Weather_Forecast weather_forecast("weather_forecast");
    weather_forecast.clk(clk);
    weather_forecast.weather_forecast(sig_weather_prediction);

    // MODULE: Fleet Management System
    FMS fms("fms");
    fms.clk(clk);
    fms.system_data(sig_fms_data);

    // stop simulation setup logging
    PRINT_LOG(18, "finish","simulation setup")

    // Start simulation
    PRINT_LOG(18, "start","simulation")
    PRINT_LOG_NEW_LINE
    sc_start(SIMULATION_DURATION_NS, SC_NS);
    PRINT_LOG_NEW_LINE
    PRINT_LOG(18, "finish","simulation")
    return 0;
}

// Utility function implementation
template <typename T, size_t N>
std::string arrayToString(const T (&arr)[N]) {
    std::ostringstream oss;
    oss << "{";
    for (size_t i = 0; i < N; ++i) {
        if (i > 0) oss << ", ";
        oss << arr[i];
    }
    oss << "}";
    return oss.str();
}

