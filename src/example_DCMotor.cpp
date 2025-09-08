#define _USE_MATH_DEFINES

#include "DCMotorSystem.hpp"
#include "SystemSimulator.hpp"
#include <cmath>

int main(int argc, char** argv)
{
    // define the DC motor system 
    DCMotorSystem dc_motor(8.464512101120487, 32.64864577071974, 27.72313746485287, 26.477893489181326);
    SystemSimulator simulator(dc_motor);

    // set the sampling time
    simulator.setDt(0.001);

    // initialize the DC motor system
    Eigen::VectorXd initial_state(2);
    initial_state << 0.0, 0.0;
    simulator.initial(initial_state, true);  // true means enable logging

    // Run the simulation for a certain number of steps
    for (size_t i = 0; i < 5001; ++i)
    {
        // step the simulation
        Eigen::VectorXd state = simulator.step();

        // print state every 10 steps
        if (i % 10 == 9) 
        {
            double time = simulator.getCurrentTime();
            std::cout << "Time: " << time << " s, State: " << state.transpose() << std::endl;
        }
    }

    // Save results to CSV file
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("DCMotorResult");

    // Reset the simulator
    // simulator.reset(initial_state);

    return 0;

}
