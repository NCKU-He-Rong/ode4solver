#include "MCKSystem.hpp"
#include "SystemSimulator.hpp"

// Example usage
int main(int argc, char** argv) 
{
    // Create an MCK system: mass = 1.0 kg, damping = 0.5 N·s/m, stiffness = 10.0 N/m
    MCKSystem mck_system(1.0, 0.5, 10.0);

    // --- Create the simulator ---
    SystemSimulator<MCKSystem> simulator(mck_system);

    // ---- Show basic system properties ----
    std::cout << "=== MCK System Properties ===" << std::endl;

    // Compute and print eigenvalues of A (characterize natural dynamics)
    Eigen::VectorXcd eigenvals = mck_system.getEigenvalues();
    std::cout << "System eigenvalues: " << eigenvals.transpose() << std::endl;

    // Print A and B matrices for verification
    std::cout << "\nSystem Matrix A:\n" << mck_system.getSystemMatrix() << std::endl;
    std::cout << "Input Matrix B: " << mck_system.getInputMatrix().transpose() << std::endl;

    
    // --- Set simulation parameters ---
    simulator.setDt(0.01);  // set simulation time step to 0.01 seconds

    // --- Initialize simulation: state = [position, velocity] = [1.0, 0.0], dt = 0.01 s ---
    Eigen::VectorXd init_state(2);
    init_state << 1.0, 0.0;                  // initial position and velocity
    simulator.initial(init_state, true);     // true means enable logging

    // Run the simulation for a certain number of steps
    std::cout << "\n=== Simulation ===" << std::endl;
    for (int i = 0; i < 10000; i++) 
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

    // --- Save results to CSV file ---
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("MCKResult");

    // Reset the simulator
    // simulator.reset(initial_state);

    return 0;
}
