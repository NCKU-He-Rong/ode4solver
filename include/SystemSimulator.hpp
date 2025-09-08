#ifndef SYSTEM_SIMULATOR_HPP
#define SYSTEM_SIMULATOR_HPP

#include <iomanip>
#include <iostream>
#include <fstream>
#include "RK4Integrator.hpp"

/**
 * @brief Modular system simulator using RK4 integration (with Eigen).
 *
 * This class is templated so it can work with any system type
 * that implements the DynamicSystem interface (e.g., MCKSystem).
 *
 * Features:
 *  - Initialization with state and sampling time
 *  - Step-by-step simulation using RK4 integration
 *  - State and time history logging
 *  - Reset functionality
 *  - CSV export of logged simulation results
 */
template<typename SystemType>
class SystemSimulator 
{
private:
    SystemType &system_;                  // The system being simulated
    double dt_;                           // Sampling time (integration step)
    bool initialized_;                    // Initialization flag
    bool log_history_ = true;             // Enable/disable history logging
    
    // Data logging
    std::vector<double> time_history_;             // Logged simulation times
    std::vector<Eigen::VectorXd> state_history_;   // Logged system states
    std::vector<Eigen::VectorXd> input_history_;   // Logged system inputs
    std::vector<Eigen::VectorXd> desire_history_;  // Logged desired trajectories

public:
    /**
     * @brief Construct the simulator with a system instance.
     *
     * @param sys System to be simulated
     */
    explicit SystemSimulator(SystemType& sys)
        : system_(sys), dt_(0.0), initialized_(false) {}
    

    /**
     * @brief Set the sampling time (dt) for the simulator.
     *
     * @param dt New sampling time (must be > 0)
     */
    void setDt(double dt) 
    {
        // Check for positive dt
        if (dt <= 0) 
        {
            throw std::invalid_argument("Sampling time must be positive");
        }
        else
        {
            dt_ = dt;
        }
    }

    /**
     * @brief Initialize the system with an initial state and sampling time.
     *
     * @param init_state Initial state vector
     */
    void initial(const Eigen::VectorXd& init_state, bool log = true) 
    {
        if (init_state.size() != system_.getStateDimension()) 
        {
            throw std::invalid_argument("Initial state dimension mismatch");
        }
        if (dt_ <= 0) 
        {
            throw std::invalid_argument("Sampling time must be positive");
        }

        // Set logging option
        log_history_ = log;
        
        // Set initial conditions
        system_.time_ = 0.0;
        system_.state_ = init_state; 
        system_.input_ = Eigen::VectorXd::Zero(system_.getInputDimension());
        system_.desire_ = Eigen::VectorXd::Zero(system_.getDesireDimension());
        initialized_ = true;
        
        // Clear history logs
        time_history_.clear();
        state_history_.clear();
        input_history_.clear();
        desire_history_.clear();
        
        // Print initialization info
        std::cout << "System initialized with dt = " << dt_ << " seconds" << std::endl;
        std::cout << "Initial state: " << system_.state_.transpose() << std::endl;
    }
    
    /**
     * @brief Perform one simulation step using RK4 integration.
     *
     * @param input Input vector (optional, defaults to empty Eigen::VectorXd)
     * @return Updated state vector after the step
     */
    Eigen::VectorXd step(const Eigen::VectorXd& desire = Eigen::VectorXd::Zero(0)) 
    {
        if (!initialized_) 
        {
            throw std::runtime_error("System not initialized. Call initial() first.");
        }

        // Update system's desired trajectory
        system_.desire_ = desire;

        // Update system input based on new time
        system_.input_ = system_.getInput(system_.time_);

        // Log current state before updating
        if (log_history_)
        {
            time_history_.push_back(system_.time_);
            state_history_.push_back(system_.state_);
            input_history_.push_back(system_.input_);
            desire_history_.push_back(system_.desire_);
        }
        
        // Integrate using RK4
        system_.state_ = RK4Integrator::integrate(
            system_, system_.state_, system_.time_, dt_);
        
        // update the timestamp
        system_.time_ += dt_;
        
        // return the updated state
        return system_.state_;
    }
    
    /**
     * @brief Reset the system to its initial state.
     */
    void reset(const Eigen::VectorXd& init_state) 
    {
        initial(init_state);
    }

    // @brief Get the simulation step size
    double getSamplingTime() const { return dt_; }

    // @brief Get the current timestamp
    double getCurrentTime() const { return system_.time_; }

    /**
     * @brief Save the simulation history to a CSV file.
     *
     * CSV Format:
     *     Time, State0, State1, ...
     *
     * @param filename Path to the output CSV file
     */
    void saveHistoryToCSV(const std::string& filename) const 
    {
        if (!log_history_) 
        {
            std::wcerr << "History logging is disabled. No data to save." << std::endl;
            return;
        }

        std::cout << "Saving history to file: " << filename << ".csv" << std::endl;

        if (time_history_.empty() || state_history_.empty()) 
        {
            std::cerr << "Warning: No history data to save!" << std::endl;
            return;
        }
        
        std::ofstream file(filename + ".csv");
        if (!file.is_open()) 
        {
            std::cerr << "Error: Cannot open file " << filename << " for writing!" << std::endl;
            return;
        }
        
        // Write CSV header
        file << "Time(Seconds)";
        int state_dim = state_history_[0].size();
        for (int i = 0; i < state_dim; i++) 
        {
            file << ",State" << i;
        }
        file << std::endl;
        
        // Write data rows
        file << std::fixed << std::setprecision(18);
        for (size_t i = 0; i < time_history_.size(); i++) 
        {
            file << time_history_[i];
            for (int j = 0; j < state_dim; j++)
            {
                file << "," << state_history_[i](j);
            }
            file << std::endl;
        }
        
        file.close();
        std::cout << "History data saved. Total " 
                  << time_history_.size() << " data points written." << std::endl;
    }
};

#endif // SYSTEM_SIMULATOR_HPP
