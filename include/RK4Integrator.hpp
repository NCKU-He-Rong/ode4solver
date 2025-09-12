#ifndef RK4_INTEGRATOR_HPP
#define RK4_INTEGRATOR_HPP

#include "DynamicSystem.hpp"

/**
 * @brief Runge-Kutta 4th-order (RK4) numerical integrator.
 *
 * This class provides a static method to propagate the state of any system
 * that implements the DynamicSystem interface using the RK4 integration scheme.
 *
 * RK4 is a popular method for solving ordinary differential equations (ODEs),
 * providing a good balance between accuracy and computational cost.
 *
 * General form:
 *
 *     x_{k+1} = x_k + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
 *
 * where
 *     k1 = f(x, u, t)
 *     k2 = f(x + dt/2 * k1, u, t + dt/2)
 *     k3 = f(x + dt/2 * k2, u, t + dt/2)
 *     k4 = f(x + dt   * k3, u, t + dt)
 */
class RK4Integrator 
{
public:
    /**
     * @brief Perform one integration step using RK4.
     *
     * @param system The dynamic system to integrate (must implement DynamicSystem)
     * @param state  Current state vector (x)
     * @param input  Input vector (u)
     * @param time   Current simulation time (t)
     * @param dt     Integration time step
     * @return Updated state vector after one RK4 step
     */
    static Eigen::VectorXd integrate(DynamicSystem& system,
                                     const Eigen::VectorXd& state,
                                     double time,
                                     double dt) 
    {
        // k1 = f(x, t)
        Eigen::VectorXd k1 = system.computeDerivatives(state, time);

        // k2 = f(x + dt/2 * k1, t + dt/2)
        Eigen::VectorXd k2 = system.computeDerivatives(state + 0.5 * dt * k1, time + 0.5 * dt);

        // k3 = f(x + dt/2 * k2, t + dt/2)
        Eigen::VectorXd k3 = system.computeDerivatives(state + 0.5 * dt * k2, time + 0.5 * dt);

        // k4 = f(x + dt * k3, t + dt)
        Eigen::VectorXd k4 = system.computeDerivatives(state + dt * k3, time + dt);

        // Final RK4 update: x(k+1) = x(k) + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        return state + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    }
};

#endif // RK4_INTEGRATOR_HPP
