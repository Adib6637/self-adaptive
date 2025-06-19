#include "weather_forecast.h"


//sc_in<bool> clk;
//sc_out<double> weather_forecast[10];


void Weather_Forecast::forecast(){

    weather_forecast[0].write(4.0); // v_wind
    weather_forecast[1].write(150.0); // theta_wind
    
}



