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
     * @param time Current simulation time (t)
     * @return Control input vector (u) at the specified time
     */
    virtual Eigen::VectorXd getInput(double time) = 0;

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
     * @brief Get the dimension of the system desired trajectory vector.
     *
     * @return Number of desired trajectory variables
     */
    virtual int getDesireDimension() const = 0;

public:
    // Current state and input vectors
    double time_;            // Current simulation time
    Eigen::VectorXd state_;  // Current state vector
    Eigen::VectorXd input_;  // Current input vector
    Eigen::VectorXd desire_; // Desired trajectory
    

};

#endif // DYNAMIC_SYSTEM_HPP
