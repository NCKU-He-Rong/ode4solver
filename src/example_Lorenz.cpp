#include "LorenzAttractor.hpp"
#include "SystemSimulator.hpp"


int main(int argc, char** argv)
{
    // Some manual settings
    bool islog = true;
    double sampling_time = 0.01;

    // Define the Lorenz attractor system
    LorenzAttractor lorenz(10.0, 28.0, 8.0/3.0);
    SystemSimulator simulator(lorenz);

    // Create the simulator and set the time step
    simulator.setDt(sampling_time);  

    // Initialize the lorenz attractor
    Eigen::VectorXd initial_state(3);
    initial_state << 1.0, 1.0, 1.0;
    simulator.initial(initial_state, islog); 

    // Run the simulation for a certain number of steps
    for (size_t i = 0; i < 3001; ++i)
    {
        // step the simulation
        Eigen::VectorXd state = simulator.step();

        // Print the current state
        double current_time = simulator.getCurrentTime();
        std::cout << "---------" << std::endl;
        std::cout << "Time: " << current_time << std::endl
                  << " State: " << state.transpose() << std::endl;
    }

    // --- Save results to CSV file ---
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("LorenzResult");

    return 0;

}


