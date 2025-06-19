#include <systemc.h>

////////////////////////////////////////////////////////////////////////////////////////////////////// Processor Module
SC_MODULE(Processor) {
    sc_in<bool> clk;
    sc_in<int> frequency;
    sc_out<double> energy;

    void compute_energy() {
        while (true) {
            wait();
            energy.write(frequency.read() * 0.01); // Simplified energy calculation
        }
    }

    SC_CTOR(Processor) {
        SC_THREAD(compute_energy);
        sensitive << clk.pos();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////// Communication Module
SC_MODULE(Communication) {
    sc_in<bool> clk;
    sc_out<double> energy;

    void compute_energy() {
        while (true) {
            wait();
            energy.write(0.05); // Simplified energy calculation
        }
    }

    SC_CTOR(Communication) {
        SC_THREAD(compute_energy);
        sensitive << clk.pos();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////// Sensor Module
SC_MODULE(Sensor) {
    sc_in<bool> clk;
    sc_out<double> energy;

    void compute_energy() {
        while (true) {
            wait();
            energy.write(0.02); // Simplified energy calculation
        }
    }

    SC_CTOR(Sensor) {
        SC_THREAD(compute_energy);
        sensitive << clk.pos();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////// Actuator Module
SC_MODULE(Actuator) {
    sc_in<bool> clk;
    sc_out<double> energy;

    void compute_energy() {
        while (true) {
            wait();
            energy.write(0.03); // Simplified energy calculation
        }
    }

    SC_CTOR(Actuator) {
        SC_THREAD(compute_energy);
        sensitive << clk.pos();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////// Main Simulation
int sc_main(int argc, char* argv[]) {
    sc_clock clk("clk", 1, SC_SEC);
    sc_signal<int> freq1, freq2;
    sc_signal<double> energy_proc1, energy_proc2, energy_comm, energy_sensor, energy_actuator;

    Processor proc1("Processor1"), proc2("Processor2");
    proc1.clk(clk);
    proc1.frequency(freq1);
    proc1.energy(energy_proc1);

    proc2.clk(clk);
    proc2.frequency(freq2);
    proc2.energy(energy_proc2);

    Communication comm("Communication");
    comm.clk(clk);
    comm.energy(energy_comm);

    Sensor sensor("Sensor");
    sensor.clk(clk);
    sensor.energy(energy_sensor);

    Actuator actuator("Actuator");
    actuator.clk(clk);
    actuator.energy(energy_actuator);

    freq1.write(100); // Set frequency for Processor 1
    freq2.write(200); // Set frequency for Processor 2

    sc_start(1000, SC_SEC); // Run simulation for 10 seconds

    std::cout << "Energy Consumption:" << std::endl;
    std::cout << "Processor 1: " << energy_proc1.read() << " units" << std::endl;
    std::cout << "Processor 2: " << energy_proc2.read() << " units" << std::endl;
    std::cout << "Communication: " << energy_comm.read() << " units" << std::endl;
    std::cout << "Sensor: " << energy_sensor.read() << " units" << std::endl;
    std::cout << "Actuator: " << energy_actuator.read() << " units" << std::endl;

    return 0;
}


//// g++ -o first_systemc main.cpp -I$SYSTEMC_HOME/include -L$SYSTEMC_HOME/lib -lsystemc -lm