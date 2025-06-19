#ifndef UAV_SENSOR_H
#define UAV_SENSOR_H

#include <systemc>
#include <string>
using namespace sc_core;

SC_MODULE(Camera) {
    sc_in<int> fps;
    sc_in<double> resolution;
    sc_in<bool> clk;
    sc_out<double> power;

    void power_consumption();

    SC_CTOR(Camera) {
        SC_METHOD(power_consumption);
        sensitive << clk.pos();
    }
};

void load_power_data(const std::string& filename);
#endif



