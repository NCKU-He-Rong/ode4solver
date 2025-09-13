#define _USE_MATH_DEFINES

#include "DCMotorSystem.hpp"
#include "SystemSimulator.hpp"
#include <cmath>

int main(int argc, char** argv)
{   
    // Some manual settings
    bool isopenloop = false;
    bool islog = true;
    double sampling_time = 0.001;

    // define the DC motor system 
    DCMotorSystem dc_motor(8.464512101120487, 
                           32.64864577071974, 
                           27.72313746485287, 
                           26.477893489181326, isopenloop);
    // set the PID gains
    dc_motor.setGains(Eigen::Vector3d(150.0, 60.0, 20.0));

    // create the simulator and set the time step
    SystemSimulator simulator(dc_motor);
    simulator.setDt(sampling_time);
    
    // initialize the DC motor system
    Eigen::VectorXd initial_state(2);
    initial_state << 0.0, 0.0;
    simulator.initial(initial_state, islog);  // true means enable logging

    // Run the simulation for a certain number of steps
    double current_time = 0.0;
    Eigen::VectorXd desire(1), desire_dot(1), desire_ddot(1);
    Eigen::VectorXd error(1), error_dot(1), error_int(1);
    
    // simulate for 5 seconds (5000 steps with dt=0.001s) 
    for (size_t i = 0; i < 5001; ++i)
    {
        // step the simulation
        Eigen::VectorXd state = simulator.step();

        // Get the current time
        current_time = simulator.getCurrentTime();

        // Get the desired trajectory and error
        if (!isopenloop)
        {
            simulator.computeDesiredAndError(desire, desire_dot, desire_ddot, 
                                             error, error_dot, error_int, 
                                             state, current_time);

            // Print the current state, desired position, and error
            std::cout << "---------" << std::endl;
            std::cout << "Time: " << current_time << std::endl
                      << " | State: " << state.transpose() << std::endl
                      << " | Desired: " << desire.transpose() << std::endl
                      << " | Desired Dot: " << desire_dot.transpose() << std::endl
                      << " | Desired DDot: " << desire_ddot.transpose() << std::endl
                      << " | Error: " << error.transpose() << std::endl
                      << " | Error Dot: " << error_dot.transpose() << std::endl
                      << " | Error Int: " << error_int.transpose() << std::endl
                      << std::endl;
        }
    }

    // Save results to CSV file
    // Must save before resetting the simulator
    simulator.saveHistoryToCSV("DCMotorResult");

    // Reset the simulator
    // initial_state << 1.0, 0.0;
    // dc_motor.setGains(Eigen::Vector3d(120.0, 30.0, 10.0));
    // simulator.reset(initial_state, islog);
    /* 
       for loop for closed-loop control 
    */
    // simulator.saveHistoryToCSV("DCMotorResult_SecondRun");


    return 0;

}
