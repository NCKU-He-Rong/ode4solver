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

    
    // --- Initialize simulation: state = [position, velocity] = [1.0, 0.0], dt = 0.01 s ---
    Eigen::VectorXd init_state(2);
    init_state << 1.0, 0.0;  // initial position and velocity
    simulator.initial(init_state, 0.01);

    // Define the input function (external force u = 0 for all time)
    mck_system.defineInput([](double time) {
        return Eigen::VectorXd::Zero(1); 
    });

    std::cout << "\n=== Simulation ===" << std::endl;

    // ---- Run 10,000 steps (100 s if dt = 0.01) ----
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < 10000; i++) 
    {
        // Print state every 10 steps
        if (i % 10 == 0) 
        {
            const auto& state = simulator.getCurrentState();
            std::cout << "t = " << simulator.getCurrentTime()
                      << ", pos = "  << state(0)
                      << ", vel = " << state(1) << std::endl;
        }

        // step the simulation
        simulator.step();
    }

    // --- Save results to CSV file ---
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("result");

    // --- Reset system back to initial conditions ---
    std::cout << "\n=== Reset System ===" << std::endl;
    simulator.reset();
    const auto& reset_state = simulator.getCurrentState();
    std::cout << "Reset state: " << reset_state.transpose() << std::endl;

    return 0;
}
