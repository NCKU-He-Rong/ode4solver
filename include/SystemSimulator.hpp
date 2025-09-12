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
    bool initialized_;                    // Initialization flag
    bool log_history_ = true;             // Enable/disable history logging
    
    // Data logging
    std::vector<double> time_history_;                  // Logged simulation times
    std::vector<Eigen::VectorXd> state_history_;        // Logged system states
    std::vector<Eigen::VectorXd> input_history_;        // Logged system inputs
    std::vector<Eigen::VectorXd> desire_history_;       // Logged desired trajectories
    std::vector<Eigen::VectorXd> desire_dot_history_;   // Logged desired derivatives
    std::vector<Eigen::VectorXd> desire_ddot_history_;  // Logged desired double derivatives
    std::vector<Eigen::VectorXd> error_history_;        // Logged errors
    std::vector<Eigen::VectorXd> error_dot_history_;    // Logged error derivatives
    std::vector<Eigen::VectorXd> error_int_history_;    // Logged error integrals

public:
    /**
     * @brief Construct the simulator with a system instance.
     *
     * @param sys System to be simulated
     */
    explicit SystemSimulator(SystemType& sys)
        : system_(sys), initialized_(false) {}
    

    /**
     * @brief Set the sampling time (dt) for the simulator.
     *
     * @param dt New sampling time (must be > 0)
     */
    void setDt(double value) 
    {
        // Check for positive dt
        if (value <= 0) 
        {
            throw std::invalid_argument("Sampling time must be positive");
        }
        else
        {
            system_.dt_ = value;
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
        if (system_.dt_ <= 0) 
        {
            throw std::invalid_argument("Sampling time must be positive");
        }

        // Set logging option
        log_history_ = log;
        
        // Set initial conditions
        system_.state_ = init_state; 
        initialized_ = true;
        
        // Clear history logs
        time_history_.clear();
        state_history_.clear();
        input_history_.clear();
        desire_history_.clear();
        desire_dot_history_.clear();
        desire_ddot_history_.clear();
        error_history_.clear();
        error_dot_history_.clear();
        error_int_history_.clear();
        
        // Print initialization info
        std::cout << "System initialized with dt = " << system_.dt_ << " seconds" << std::endl;
        std::cout << "Initial state: " << system_.state_.transpose() << std::endl;
    }
    
    /**
     * @brief Perform one simulation step using RK4 integration.
     *
     * @param input Input vector (optional, defaults to empty Eigen::VectorXd)
     * @return Updated state vector after the step
     */
    Eigen::VectorXd step()
    {
        if (!initialized_) 
        {
            throw std::runtime_error("System not initialized. Call initial() first.");
        }

        // Compute desired trajectory and error
        computeDesiredAndError(system_.desire_, 
                               system_.desire_dot_, 
                               system_.desire_ddot_,
                               system_.error_, 
                               system_.error_dot_,
                               system_.error_int_,
                               system_.state_, 
                               system_.time_);

        // Update system input based on new time
        system_.input_ = system_.getInput(system_.state_, system_.time_);
        
        // Log current state before updating
        if (log_history_)
        {
            time_history_.push_back(system_.time_);
            state_history_.push_back(system_.state_);
            input_history_.push_back(system_.input_);

            if (!system_.isopenloop_)
            {
                desire_history_.push_back(system_.desire_);
                desire_dot_history_.push_back(system_.desire_dot_);
                desire_ddot_history_.push_back(system_.desire_ddot_);
                error_history_.push_back(system_.error_);
                error_dot_history_.push_back(system_.error_dot_);
                error_int_history_.push_back(system_.error_int_);
            }
        }
        
        // Integrate using RK4
        system_.state_ = RK4Integrator::integrate(
            system_, system_.state_, system_.time_, system_.dt_);
        
        // update the timestamp
        system_.time_ += system_.dt_;
        
        // return the updated state
        return system_.state_;
    }
    
    void computeDesiredAndError(Eigen::VectorXd& desire, 
                                Eigen::VectorXd& desire_dot, 
                                Eigen::VectorXd& desire_ddot,
                                Eigen::VectorXd& error, 
                                Eigen::VectorXd& error_dot,
                                Eigen::VectorXd& error_int,
                                const Eigen::VectorXd& state,
                                double time)
    {
        system_.computeDesiredAndError(desire, 
                                       desire_dot, 
                                       desire_ddot,
                                       error, 
                                       error_dot,
                                       error_int,
                                       state, 
                                       time);
    }

    /**
     * @brief Reset the system to its initial state.
     */
    void reset(const Eigen::VectorXd& init_state, bool log = true) 
    {
        system_.reset();
        initial(init_state, log);
    }

    // @brief Get the simulation step size
    double getSamplingTime() const { return system_.dt_; }

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

        // Get dimensions
        int state_dim = 0;
        int input_dim = 0;
        int desire_dim = 0;
        int desire_dot_dim = 0;
        int desire_ddot_dim = 0;
        int error_dim = 0;
        int error_dot_dim = 0;
        int error_int_dim = 0;

        // Write CSV header (time and state)
        file << "Time(Seconds)";
        state_dim = state_history_[0].size();
        for (int i = 0; i < state_dim; i++) 
        {
            file << ",State" << i;
        }
        
        // Write CSV header (input)
        input_dim = input_history_[0].size();
        for (int i = 0; i < input_dim; i++) 
        {
            file << ",Input" << i;
        }

        // Write CSV header (desired trajectory and error, if closed-loop)
        if (!system_.isopenloop_)
        {
            desire_dim = desire_history_[0].size();
            for (int i = 0; i < desire_dim; i++) 
            {
                file << ",Desire" << i;
            }
            
            // Write CSV header (desired trajectory derivative)
            desire_dot_dim = desire_dot_history_[0].size();
            for (int i = 0; i < desire_dot_dim; i++) 
            {
                file << ",Desire_dot" << i;
            }

            // Write CSV header (desired trajectory double derivative)
            desire_ddot_dim = desire_ddot_history_[0].size();
            for (int i = 0; i < desire_ddot_dim; i++) 
            {
                file << ",Desire_ddot" << i;
            }

            // Write CSV header (error)
            error_dim = error_history_[0].size();
            for (int i = 0; i < error_dim; i++) 
            {
                file << ",Error" << i;
            }

            // Write CSV header (error derivative)
            error_dot_dim = error_dot_history_[0].size();
            for (int i = 0; i < error_dot_dim; i++) 
            {
                file << ",Error_dot" << i;
            }

            // Write CSV header (error integral)
            error_int_dim = error_int_history_[0].size();
            for (int i = 0; i < error_int_dim; i++) 
            {
                file << ",Error_int" << i;
            }
        }
        file << std::endl;

        // Write data (time and state)
        file << std::fixed << std::setprecision(18);
        for (size_t i = 0; i < time_history_.size(); i++) 
        {   
            // time 
            file << time_history_[i];

            // state
            for (int j = 0; j < state_dim; j++)
            {
                file << "," << state_history_[i](j);
            }

            // input
            for (int j = 0; j < input_dim; j++)
            {
                file << "," << input_history_[i](j);
            }

            if (!system_.isopenloop_) 
            {
                // desired trajectory
                for (int j = 0; j < desire_dim; j++)
                {
                    file << "," << desire_history_[i](j);
                }

                // desired trajectory derivative
                for (int j = 0; j < desire_dot_dim; j++)
                {
                    file << "," << desire_dot_history_[i](j);
                }

                // desired trajectory double derivative
                for (int j = 0; j < desire_ddot_dim; j++)
                {
                    file << "," << desire_ddot_history_[i](j);
                }

                // error
                for (int j = 0; j < error_dim; j++)
                {
                    file << "," << error_history_[i](j);
                }

                // error derivative
                for (int j = 0; j < error_dot_dim; j++)
                {
                    file << "," << error_dot_history_[i](j);
                }

                // error integral
                for (int j = 0; j < error_int_dim; j++)
                {
                    file << "," << error_int_history_[i](j);
                }
            }
            file << std::endl;
        }
        
        file.close();
        std::cout << "History data saved. Total " 
                  << time_history_.size() << " data points written." << std::endl;
    }
};

#endif // SYSTEM_SIMULATOR_HPP
