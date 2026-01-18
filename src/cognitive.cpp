#include "cognitive.h"    

// cognitive behaviour: update output port data
void Cognitive::run(){
    for (int i = 0; i < 20; ++i) {
        power_consumption[i].write(sig_power_consumption[i]);
        operation_time[i].write(sig_operation_time[i]);
    }
}