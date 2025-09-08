#ifndef DC_MOTOR_HPP
#define DC_MOTOR_HPP

#include "DynamicSystem.hpp"

class DCMotorSystem : public DynamicSystem 
{
public:     
    DCMotorSystem(double a, double b, double c_pos, double c_neg):
        a_(a), b_(b), c_pos_(c_pos), c_neg_(c_neg) {}

    Eigen::VectorXd computeDerivatives(const Eigen::VectorXd& state, double time) override 
    {
        if (state.size() != 2) 
        {
            throw std::invalid_argument("DC Motor system requires 2-dimensional state vector");
        }

        Eigen::VectorXd derivatives(2);
        derivatives[0] = state[1];
        derivatives[1] = - a_ * state[1] + b_ * getInput(time)[0]
                         - c_pos_ * (state[1] > 0 ? 1.0 : 0.0) - c_neg_ * (state[1] < 0 ? -1.0 : 0.0);
        return derivatives;
    }

    Eigen::VectorXd getInput(double time) override 
    {
        Eigen::VectorXd u(1);
        u(0) = 6.0 * sin(2 * M_PI * 1.0 * time);
        return u;
    }

    int getStateDimension() const override {return 2;}
    int getInputDimension() const override {return 1;}
    int getDesireDimension() const override {return 1;}

    Eigen::VectorXd getParameters() const
    {
        Eigen::VectorXd parameters(4);
        parameters << a_, b_, c_pos_, c_neg_;
        return parameters;
    }

    void setParameters(const Eigen::VectorXd& parameters)
    {
        if (parameters.size() != 4) 
        {
            throw std::invalid_argument("DC Motor system requires 4 parameters");
        }
        a_ = parameters[0];
        b_ = parameters[1];
        c_pos_ = parameters[2];
        c_neg_ = parameters[3];
    }

private:
    double a_;
    double b_;
    double c_pos_;
    double c_neg_;
};

#endif // DC_MOTOR_HPP