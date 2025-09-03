#include "DCMotorSystem.hpp"
#include "SystemSimulator.hpp"

#define PI 3.1415926535897932384626433832795

int main(int argc, char** argv)
{
    // define the DC motor system
    DCMotorSystem dc_motor(8.464512101120487, 32.64864577071974, 27.72313746485287, 26.477893489181326);
    SystemSimulator simulator(dc_motor);

    // initialize the DC motor system
    Eigen::VectorXd initial_state(2);
    initial_state << 0.0, 0.0;
    simulator.initial(initial_state, 0.01);

    // Run the simulation for a certain number of steps
    for (size_t i = 0; i <= 5000; ++i)
    {
        Eigen::VectorXd u(1);
        u(0) = 6.0 * sin(2 * PI * 1.0 * i * 0.01);
        simulator.step(u);
    }

    // --- Save results to CSV file ---
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("DCMotorResult");

    //
    return 0;

}
