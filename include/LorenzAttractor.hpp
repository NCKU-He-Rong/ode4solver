#define _USE_MATH_DEFINES

#ifndef LORENZ_ATTRACTOR_HPP
#define LORENZ_ATTRACTOR_HPP

#include "DynamicSystem.hpp"

class LorenzAttractor : public DynamicSystem 
{
public:     
    LorenzAttractor(double sigma, double rho, double beta):
        sigma_(sigma), rho_(rho), beta_(beta)
    {
        // Define the control mode (open-loop for this system)
        isopenloop_ = true;

        // Initialize sampling time
        dt_ = 0.0;

        // Reset the system
        reset();
    }

    Eigen::VectorXd computeDerivatives(const Eigen::VectorXd& state, double time) override 
    {   
        // Check state dimension
        if (state.size() != getStateDimension()) 
        {
            throw std::invalid_argument("DC Motor system requires a state vector of size" + std::to_string(getStateDimension()));
        }

        // Compute state derivatives
        Eigen::VectorXd derivatives(getStateDimension());
        derivatives[0] = sigma_ * (state[1] - state[0]);
        derivatives[1] = state[0] * (rho_ - state[2]) - state[1];
        derivatives[2] = state[0] * state[1] - beta_ * state[2];
        return derivatives;
    }
    
    // No control input for this system
    Eigen::VectorXd getInput(const Eigen::VectorXd& state, double time) override 
    {   
        // Temporary input vector
        Eigen::VectorXd input_temp(0);
        return input_temp;
    }

    // Desired trajectory and error computation (because this is an open-loop system, these are not used)
    virtual void computeDesiredAndError(Eigen::VectorXd& desire, 
                                        Eigen::VectorXd& desire_dot, 
                                        Eigen::VectorXd& desire_ddot,
                                        Eigen::VectorXd& error, 
                                        Eigen::VectorXd& error_dot,
                                        Eigen::VectorXd& error_int,
                                        const Eigen::VectorXd& state,
                                        double time) override
    {return;}

    // Get the system state and input dimensions
    int getStateDimension() const override {return 3;}
    int getInputDimension() const override {return 0;}

    Eigen::VectorXd getParameters() const
    {
        Eigen::VectorXd parameters(3);
        parameters << sigma_, beta_, rho_;
        return parameters;
    }

    void setParameters(const Eigen::VectorXd& parameters)
    {
        if (parameters.size() != 3) 
        {
            throw std::invalid_argument("Lorenz Attractor system requires 3 parameters");
        }
        sigma_ = parameters[0];
        beta_ = parameters[1];
        rho_ = parameters[2];
    }

    void reset() override
    {
        // Reset timestamp
        time_ = 0.0;
       
        // Initialize state and input vectors
        state_ = Eigen::VectorXd::Zero(3);
        input_.resize(0);

        desire_.resize(0);
        desire_dot_.resize(0);
        desire_ddot_.resize(0);

        error_.resize(0);
        error_dot_.resize(0);
        error_int_.resize(0);
    }

private:
    double sigma_;
    double beta_;
    double rho_;
};

#endif // LORENZ_ATTRACTOR_HPP