#include "monitor.h"

// monitor behaviout: foward data from fms to observed data interface
void Monitor::monitor(){
    for (int i = 0; i < 20; ++i) {
        observed_data[i].write(fms_data[i].read());
    }
}



