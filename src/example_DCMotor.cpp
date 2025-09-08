#define _USE_MATH_DEFINES

#include "DCMotorSystem.hpp"
#include "SystemSimulator.hpp"
#include <cmath>

int main(int argc, char** argv)
{
    // define the DC motor system 
    DCMotorSystem dc_motor(8.464512101120487, 32.64864577071974, 27.72313746485287, 26.477893489181326);
    SystemSimulator simulator(dc_motor);

    // initialize the DC motor system
    Eigen::VectorXd initial_state(2);
    initial_state << 0.0, 0.0;
    simulator.initial(initial_state, 0.001);

    // Define the input function (sinusoidal input)
    dc_motor.defineInput([](double time) {
        Eigen::VectorXd u(1);
        u(0) = 6.0 * sin(2 * M_PI * 1.0 * time);
        return u;
    });

    // Run the simulation for a certain number of steps
    for (size_t i = 0; i < 5000; ++i)
    {
        simulator.step();
    }

    // --- Save results to CSV file ---
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("DCMotorResult");

    //
    return 0;

}
