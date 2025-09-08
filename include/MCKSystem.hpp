#ifndef MCK_SYSTEM_HPP
#define MCK_SYSTEM_HPP

// Include the base class
#include "DynamicSystem.hpp"

/**
 * @brief Mass-Spring-Damper (MCK) system implementation.
 *
 * This class models a standard second-order linear system:
 *
 *     m * x'' + c * x' + k * x = u
 *
 * which can be expressed in state-space form:
 *
 *     d/dt [x, x']^T = A * [x, x']^T + B * u
 *
 * where
 *     A = [[ 0,        1 ],
 *          [ -k/m,  -c/m ]]
 *
 *     B = [[ 0   ],
 *          [ 1/m ]]
 */
class MCKSystem : public DynamicSystem 
{  
public:
    /**
     * @brief Constructor for the MCK system.
     *
     * @param m Mass (must be > 0)
     * @param c Damping coefficient
     * @param k Stiffness coefficient
     */
    MCKSystem(double m, double c, double k) 
        : mass(m), damping(c), stiffness(k) 
    {
        if (m <= 0) throw std::invalid_argument("Mass must be positive");
        updateSystemMatrices();
    }
    
    /**
     * @brief Compute the state derivative (x_dot).
     *
     * Implements:
     *     dx/dt = A * x + B * u
     *
     * @param state Current state vector [x, x']
     * @param time  Current simulation time 
     * @return State derivative vector [x', x'']
     */
    Eigen::VectorXd computeDerivatives(const Eigen::VectorXd& state,
                                       double time) override 
    {
        if (state.size() != 2) 
        {
            throw std::invalid_argument("MCK system requires 2-dimensional state vector");
        }

        return A_matrix * state + B_vector * getInput(time);
    }

    Eigen::VectorXd getInput(double time) override 
    {
        // Zero input
        return Eigen::VectorXd::Zero(1);
    }
    
    /// @brief Dimension of the state vector (always 2: [x, x'])
    int getStateDimension() const override { return 2; }

    /// @brief Dimension of the input vector (always 1: [u])
    int getInputDimension() const override { return 1; }

    /// @brief Dimension of the desired trajectory vector (always 0)
    int getDesireDimension() const override { return 0; }

    /// @brief Get the system matrix A
    const Eigen::Matrix2d& getSystemMatrix() const { return A_matrix; }

    /// @brief Get the input matrix B
    const Eigen::Vector2d& getInputMatrix() const { return B_vector; }

    /// @brief Get system parameters [mass, damping, stiffness]
    Eigen::VectorXd getParameters() const 
    {
        Eigen::VectorXd params(3);
        params << mass, damping, stiffness;
        return params;
    }
    
    /// @brief Compute eigenvalues of the system matrix A
    Eigen::VectorXcd getEigenvalues() const { return A_matrix.eigenvalues(); }

    /**
     * @brief Set system parameters [mass, damping, stiffness].
     *
     * Automatically updates A and B matrices.
     *
     * @param params Vector of size 3: [mass, damping, stiffness]
     */
    void setParameters(const Eigen::VectorXd& params) 
    {
        if (params.size() != 3) 
        {
            throw std::invalid_argument("MCK system requires 3 parameters: [mass, damping, stiffness]");
        }
        mass = params(0);
        damping = params(1);
        stiffness = params(2);
        if (mass <= 0) throw std::invalid_argument("Mass must be positive");
        updateSystemMatrices();
    }

private:
    double mass;               ///< Mass (m)
    double damping;            ///< Damping coefficient (c)
    double stiffness;          ///< Stiffness coefficient (k)

    // State-space representation matrices
    Eigen::Matrix2d A_matrix;  ///< System dynamics matrix (A)
    Eigen::Vector2d B_vector;  ///< Input matrix (B)
    
    /**
     * @brief Update the system matrices (A and B) when parameters change.
     */
    void updateSystemMatrices() 
    {
        A_matrix << 0.0,            1.0,
                   -stiffness/mass, -damping/mass;
        B_vector << 0.0, 1.0/mass;
    }
};

#endif // MCK_SYSTEM_HPP
