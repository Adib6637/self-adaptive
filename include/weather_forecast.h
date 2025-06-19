#ifndef WEATHER_FORECAST_H
#define WEATHER_FORECAST_H

#include <systemc>
#include <string>
#include <vector>

using namespace sc_core;

SC_MODULE(Weather_Forecast) {
    sc_in<bool> clk;
    sc_core::sc_vector<sc_out<double>> weather_forecast;

    void forecast();

    SC_CTOR(Weather_Forecast)
        : weather_forecast("weather_forecast", 20)
    {
        SC_METHOD(forecast);
        sensitive << clk.pos();
    }
};

#endif
