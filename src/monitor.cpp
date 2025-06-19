#include "monitor.h"

//sc_in<bool> clk;
//sc_in<double> managed_system_data[10];
//sc_in<double> new_cfg[10];
//sc_out<double> observed_data[10];


void Monitor::monitor(){

    for (int i = 0; i < 20; ++i) {
        observed_data[i].write(managed_system_data[i].read());
        //std::cout<< observed_data[i] <<std::endl;
    }
}



