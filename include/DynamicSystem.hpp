#ifndef DYNAMIC_SYSTEM_HPP
#define DYNAMIC_SYSTEM_HPP

// Include the Eigen library for vector and matrix operations
#include <Eigen/Dense>

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
     *     dx/dt = f(x, u, t)
     *
     * @param state Current state vector (x)
     * @param input Control input vector (u)
     * @param time  Current simulation time (t)
     * @return A vector representing the time derivative of the state (dx/dt)
     */
    virtual Eigen::VectorXd computeDerivatives(const Eigen::VectorXd& state, 
                                               const Eigen::VectorXd& input, 
                                               double time) const = 0;
    
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
};

#endif // DYNAMIC_SYSTEM_HPP
