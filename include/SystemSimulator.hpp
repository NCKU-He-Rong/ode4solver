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
    SystemType &system_;                 ///< The system being simulated
    Eigen::VectorXd current_state_;      ///< Current system state
    Eigen::VectorXd initial_state_;      ///< Initial system state
    double current_time_;                ///< Current simulation time
    double dt_;                          ///< Sampling time (integration step)
    bool initialized_;                   ///< Initialization flag
    
    // Data logging
    std::vector<double> time_history_;             ///< Logged simulation times
    std::vector<Eigen::VectorXd> state_history_;   ///< Logged system states
    
public:
    /**
     * @brief Construct the simulator with a system instance.
     *
     * @param sys System to be simulated
     */
    explicit SystemSimulator(SystemType& sys)
        : system_(sys), current_time_(0.0), dt_(0.0), initialized_(false) {}
    
    /**
     * @brief Initialize the system with an initial state and sampling time.
     *
     * @param init_state Initial state vector
     * @param sampling_time Simulation step size (must be > 0)
     */
    void initial(const Eigen::VectorXd& init_state, double sampling_time) 
    {
        if (init_state.size() != system_.getStateDimension()) 
        {
            throw std::invalid_argument("Initial state dimension mismatch");
        }
        if (sampling_time <= 0) 
        {
            throw std::invalid_argument("Sampling time must be positive");
        }
        
        initial_state_ = init_state;
        current_state_ = init_state;
        dt_ = sampling_time;
        current_time_ = 0.0;
        initialized_ = true;
        
        // Clear history logs
        time_history_.clear();
        state_history_.clear();

        // Log the first state and time
        time_history_.push_back(current_time_);
        state_history_.push_back(current_state_);
        
        std::cout << "System initialized with dt = " << dt_ << " seconds" << std::endl;
        std::cout << "Initial state: " << current_state_.transpose() << std::endl;
    }
    
    /**
     * @brief Perform one simulation step using RK4 integration.
     *
     * @param input Input vector (optional, defaults to empty Eigen::VectorXd)
     */
    void step(const Eigen::VectorXd& input = Eigen::VectorXd()) 
    {
        if (!initialized_) 
        {
            throw std::runtime_error("System not initialized. Call initial() first.");
        }
        
        // Integrate using RK4
        current_state_ = RK4Integrator::integrate(
            system_, current_state_, input, current_time_, dt_);
        
        current_time_ += dt_;

        // Log current state before updating
        time_history_.push_back(current_time_);
        state_history_.push_back(current_state_);
    }
    
    /**
     * @brief Reset the system to its initial state.
     */
    void reset() 
    {
        if (!initialized_) 
        {
            throw std::runtime_error("System not initialized. Call initial() first.");
        }
        
        current_state_ = initial_state_;
        current_time_ = 0.0;
        
        // Clear history logs
        time_history_.clear();
        state_history_.clear();

        // Reset initialization flag
        initialized_ = false; 

        std::cout << "System reset to initial conditions" << std::endl;
    }
    
    /// @brief Get the current state vector
    const Eigen::VectorXd& getCurrentState() const { return current_state_; }
    
    /// @brief Get the current simulation time
    double getCurrentTime() const { return current_time_; }
    
    /// @brief Get the simulation step size
    double getSamplingTime() const { return dt_; }
    
    /// @brief Get the logged time history
    const std::vector<double>& getTimeHistory() const { return time_history_; }
    
    /// @brief Get the logged state history
    const std::vector<Eigen::VectorXd>& getStateHistory() const { return state_history_; }
    
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
        file << std::fixed << std::setprecision(8);
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
