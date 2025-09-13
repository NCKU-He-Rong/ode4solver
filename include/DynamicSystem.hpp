#define _USE_MATH_DEFINES

#ifndef DYNAMIC_SYSTEM_HPP
#define DYNAMIC_SYSTEM_HPP

// Include the Eigen library for vector and matrix operations
#include <Eigen/Dense>

// Include the functional header for std::function
#include <functional>

/**
 * @brief Abstract base class that defines the interface for a dynamic system.
 *
 * This class represents a general continuous-time dynamic system, where the 
 * evolution of the system state is governed by differential equations of the form:
 *
 *     dx/dt = f(x, u, t)
 *
 * Subclasses (e.g., spring-damper system, robotic dynamics, etc.) must implement 
 * the pure virtual functions defined here.
 */
class DynamicSystem 
{
public:
    // Virtual destructor to ensure proper cleanup of derived classes
    virtual ~DynamicSystem() = default;
    
    /**
     * @brief Compute the state derivatives of the system.
     *
     * This represents the system dynamics equation:
     *     dx/dt = f(x, t)
     *
     * @param state Current state vector (x)
     * @param time  Current simulation time (t)
     * @return A vector representing the time derivative of the state (dx/dt)
     */
    virtual Eigen::VectorXd computeDerivatives(const Eigen::VectorXd& state, double time) = 0;
    
    /**
     * @brief Get the control input vector at a given time.
     * 
     * @param state Current state vector (x)
     * @param time Current simulation time (t)
     * @return Control input vector (u) at the specified time
     */
    virtual Eigen::VectorXd getInput(const Eigen::VectorXd& state, double time) = 0;

    /**
     * @brief Compute the desired trajectory and tracking error.
     *
     * This function computes the desired trajectory (position, velocity, acceleration)
     * and the tracking error (error, error derivative, error integral) based on the 
     * current state and time.
     *
     * @param desire Output vector for desired trajectory
     * @param desire_dot Output vector for derivative of desired trajectory
     * @param desire_ddot Output vector for second derivative of desired trajectory
     * @param error Output vector for tracking error
     * @param error_dot Output vector for derivative of tracking error
     * @param error_int Output vector for integral of tracking error
     * @param state Current state vector (x)
     * @param time Current simulation time (t)
     */
    virtual void computeDesiredAndError(Eigen::VectorXd& desire, 
                                        Eigen::VectorXd& desire_dot, 
                                        Eigen::VectorXd& desire_ddot,
                                        Eigen::VectorXd& error, 
                                        Eigen::VectorXd& error_dot,
                                        Eigen::VectorXd& error_int,
                                        const Eigen::VectorXd& state,
                                        double time) = 0;

    /**
     * @brief Get the dimension of the system state vector.
     *
     * @return Number of state variables
     */
    virtual int getStateDimension() const = 0;
    
    /**
     * @brief Get the dimension of the system input vector.
     *
     * @return Number of input variables
     */
    virtual int getInputDimension() const = 0;

    /**
     * @brief Reset the system to its initial state.
     */
    virtual void reset() = 0;

public:
    // Current state and input vectors
    double time_;                  // Current simulation time
    double dt_;                     // Time step for simulation
    bool isopenloop_;               // Flag indicating if the system is in open-loop mode

    Eigen::VectorXd state_;        // Current state vector
    Eigen::VectorXd input_;         // Current input vector

    Eigen::VectorXd desire_;       // Desired trajectory
    Eigen::VectorXd desire_dot_;   // Derivative of desired trajectory
    Eigen::VectorXd desire_ddot_;  // Second derivative of desired trajectory

    Eigen::VectorXd error_;        // Tracking error
    Eigen::VectorXd error_dot_;    // Derivative of tracking error
    Eigen::VectorXd error_int_;    // Integral of tracking error
};

#endif // DYNAMIC_SYSTEM_HPP
