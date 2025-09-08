#ifndef LORENZ_ATTRACTOR_HPP
#define LORENZ_ATTRACTOR_HPP

#include "DynamicSystem.hpp"

class LorenzAttractor : public DynamicSystem 
{
public:     
    LorenzAttractor(double sigma, double beta, double rho):
        sigma_(sigma), beta_(beta), rho_(rho) {}

    Eigen::VectorXd computeDerivatives(const Eigen::VectorXd& state, double time) const override 
    {
        if (state.size() != 3) 
        {
            throw std::invalid_argument("Lorenz Attractor system requires 3-dimensional state vector");
        }

        Eigen::VectorXd derivatives(3);
        derivatives[0] = sigma_ * (state[1] - state[0]);
        derivatives[1] = state[0] * (rho_ - state[2]) - state[1];
        derivatives[2] = state[0] * state[1] - beta_ * state[2];
        return derivatives;
    }

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

private:
    double sigma_;
    double beta_;
    double rho_;
};

#endif // LORENZ_ATTRACTOR_HPP