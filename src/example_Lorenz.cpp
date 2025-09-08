#include "LorenzAttractor.hpp"
#include "SystemSimulator.hpp"


int main(int argc, char** argv)
{
    // define the Lorenz attractor system
    LorenzAttractor lorenz(10.0, 28.0, 8.0/3.0);
    SystemSimulator simulator(lorenz);

    // initialize the lorenz attractor
    Eigen::VectorXd initial_state(3);
    initial_state << 1.0, 1.0, 1.0;
    simulator.initial(initial_state, 0.01);

    // Define the input function (external force u = 0 for all time)
    lorenz.defineInput([](double time) {
        return Eigen::VectorXd::Zero(1);
    });

    // Run the simulation for a certain number of steps
    for (size_t i = 0; i < 100000000; ++i)
    {
        simulator.step();
    }

    // --- Save results to CSV file ---
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("result");

    //
    return 0;

}


