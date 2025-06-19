#include "UAV_sensor.h"
#include <systemc>
#include <iostream>

using namespace sc_core;

int sc_main(int argc, char* argv[]) {
    // Signals
    sc_signal<int> fps_signal;
    sc_signal<double> resolution_signal;
    sc_signal<bool> clk_signal;
    sc_signal<double> power_signal;

    // Instantiate the Camera module
    Camera camera("camera");

    // Connect signals to ports
    camera.fps(fps_signal);
    camera.resolution(resolution_signal);
    camera.clk(clk_signal);
    camera.power(power_signal);

    // Load power data from CSV file
    load_power_data("../power_data.csv");

    // Open VCD file for waveform output
    sc_trace_file *wf = sc_create_vcd_trace_file("camera_power_consumption");
    sc_trace(wf, fps_signal, "fps");
    sc_trace(wf, resolution_signal, "resolution");
    sc_trace(wf, clk_signal, "clk");
    sc_trace(wf, power_signal, "power");

    // Test values matching the CSV file
    int fps_values[] = {30, 60, 120, 30, 30};
    double resolution_values[] = {1080.0, 720.0, 480.0, 720.0, 1080.0};

    // Simulate a few clock cycles with different fps and resolution values
    for (int i = 0; i < 5; ++i) {
        fps_signal.write(fps_values[i]);
        resolution_signal.write(resolution_values[i]);
        clk_signal.write(true);
        sc_start(1, SC_NS);
        clk_signal.write(false);
        sc_start(1, SC_NS);
        std::cout << "At time " << sc_time_stamp() << ", fps: " << fps_signal.read()
                  << ", resolution: " << resolution_signal.read()
                  << ", power: " << power_signal.read() << std::endl;
    }

    // Close VCD file
    sc_close_vcd_trace_file(wf);

    return 0;
}
