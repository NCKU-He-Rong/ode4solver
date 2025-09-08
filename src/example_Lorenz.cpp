#include "LorenzAttractor.hpp"
#include "SystemSimulator.hpp"


int main(int argc, char** argv)
{
    // define the Lorenz attractor system
    LorenzAttractor lorenz(10.0, 28.0, 8.0/3.0);
    SystemSimulator simulator(lorenz);

    // --- Set simulation parameters ---
    simulator.setDt(0.01);  // set simulation time step to 0.01 seconds

    // initialize the lorenz attractor
    Eigen::VectorXd initial_state(3);
    initial_state << 1.0, 1.0, 1.0;
    simulator.initial(initial_state, true);  // true means enable logging

    // Run the simulation for a certain number of steps
    for (size_t i = 0; i < 100000; ++i)
    {
        // step the simulation
        Eigen::VectorXd state = simulator.step();

        // print state every 100 steps
        if (i % 100 == 99) 
        {
            double time = simulator.getCurrentTime();
            std::cout << "Time: " << time << " s, State: " << state.transpose() << std::endl;
        }
    }

    // --- Save results to CSV file ---
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("LorenzResult");

    // Reset the simulator
    // simulator.reset(initial_state);

    //
    return 0;

}


