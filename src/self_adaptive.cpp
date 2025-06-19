#include "self_adaptive.h"    

//sc_in<double> managed_system_data[10];
//sc_in<double> weather_prediction[10];
//sc_out<double> power_consumption[10];
//sc_out<double> operation_time[10];

void Self_Adaptive::run(){
    for (int i = 0; i < 20; ++i) {
        operation_time[i].write(sig_operation_time[i]);
        operation_time[i].write(sig_operation_time[i]);
    }
    //for (int i = 0; i < 20; ++i) {
    //    std::cout<< managed_system_data[i] <<std::endl;
    //}
}