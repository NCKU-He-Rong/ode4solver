#define _USE_MATH_DEFINES

#ifndef DC_MOTOR_HPP
#define DC_MOTOR_HPP

#include "DynamicSystem.hpp"
#include <iostream>

class DCMotorSystem : public DynamicSystem 
{
public:     
    DCMotorSystem(double a, double b, double c_pos, double c_neg, bool isopenloop):
        a_(a), b_(b), c_pos_(c_pos), c_neg_(c_neg)
    {
        // Initialize the parameters
        alpha = (a_ / b_);
        beta = (1.0 / b_);
        gamma1 = (c_pos_ / b_);
        gamma2 = (c_neg_ / b_);

        // Define the control mode
        isopenloop_ = isopenloop;

        // Initialize sampling time
        dt_ = 0.0;

        // Initialize gains
        kp_ = 0.0;
        ki_ = 0.0;
        kd_ = 0.0;

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
        derivatives[0] = state[1];
        derivatives[1] = - a_ * state[1] + b_ * getInput(state, time)[0]
                         - c_pos_ * (state[1] > 0 ? 1.0 : 0.0) - c_neg_ * (state[1] < 0 ? -1.0 : 0.0);

        // return the state derivatives (dx/dt)
        return derivatives;
    }
    
    Eigen::VectorXd getInput(const Eigen::VectorXd& state, double time) override 
    {   
        // Temporary input vector
        Eigen::VectorXd input_temp(1);

        // Compute open-loop input
        if (isopenloop_)
        {
            input_temp(0) = 6.0 * sin(2 * M_PI * 1.0 * time);
            return input_temp;
        }
        // Compute closed-loop input
        else
        {
            // Temporary desired and error vectors
            Eigen::VectorXd desire_temp(1), desire_dot_temp(1), desire_ddot_temp(1);
            Eigen::VectorXd error_temp(1), error_dot_temp(1), error_int_temp(1);

            // Compute desired and error
            computeDesiredAndError(desire_temp, desire_dot_temp, desire_ddot_temp, 
                                   error_temp, error_dot_temp, error_int_temp, 
                                   state, time);

            // PID terms
            Eigen::VectorXd u_pid(1);
            u_pid(0) = beta * (kp_ * error_temp(0) + kd_ * error_dot_temp(0) + ki_ * error_int_temp(0));

            // Feedforward terms
            Eigen::VectorXd u_ff(1);
            u_ff(0) = alpha * state(1) + beta * desire_ddot_temp(0) + gamma1 * (state(1) > 0 ? 1.0 : 0.0) + gamma2 * (state(1) < 0 ? -1.0 : 0.0);

            // Combine PID and feedforward terms
            input_temp = u_pid + u_ff;
            return input_temp;
        }
    }

    virtual void computeDesiredAndError(Eigen::VectorXd& desire, 
                                        Eigen::VectorXd& desire_dot, 
                                        Eigen::VectorXd& desire_ddot,
                                        Eigen::VectorXd& error, 
                                        Eigen::VectorXd& error_dot,
                                        Eigen::VectorXd& error_int,
                                        const Eigen::VectorXd& state,
                                        double time) override
    {
        if (isopenloop_)
        {
            return;
        }
        else
        {
            // Desired terms
            double frequency = 1.0;
            desire(0) = (M_PI / 4.0) * (1 + sin(2 * M_PI * frequency * time - M_PI / 2.0));
            desire_dot(0) = (M_PI / 4.0) * (2 * M_PI * frequency) * cos(2 * M_PI * frequency * time - M_PI / 2.0);
            desire_ddot(0) = - (M_PI / 4.0) * (2 * M_PI * frequency) * (2 * M_PI * frequency) * sin(2 * M_PI * frequency * time - M_PI / 2.0);
            
            // Error terms
            error(0) = desire(0) - state(0);
            error_dot(0) = desire_dot(0) - state(1);
            error_int(0) = error_int_(0) + error(0) * dt_;
        }
    }

    void setGains(const Eigen::VectorXd & gains)
    {
        kp_ = gains(0);
        ki_ = gains(1);
        kd_ = gains(2);
        
        if (gains.size() > 3)
        {
            alpha = gains(3);
            beta = gains(4);
            gamma1 = gains(5);
            gamma2 = gains(6);
        }
    }

    int getStateDimension() const override {return 2;}
    int getInputDimension() const override {return 1;}

    Eigen::VectorXd getParameters() const
    {
        Eigen::VectorXd parameters(4);
        parameters << a_, b_, c_pos_, c_neg_;
        return parameters;
    }

    void reset() override
    {
        // Reset timestamp
        time_ = 0.0;
       
        // Initialize state and input vectors
        if (isopenloop_)
        {
            state_ = Eigen::VectorXd::Zero(2);
            input_ = Eigen::VectorXd::Zero(1);

            desire_.resize(0);
            desire_dot_.resize(0);
            desire_ddot_.resize(0);

            error_.resize(0);
            error_dot_.resize(0);
            error_int_.resize(0);
        }
        else
        {
            state_ = Eigen::VectorXd::Zero(2);
            input_ = Eigen::VectorXd::Zero(1);

            desire_ = Eigen::VectorXd::Zero(1);
            desire_dot_ = Eigen::VectorXd::Zero(1);
            desire_ddot_ = Eigen::VectorXd::Zero(1);

            error_ = Eigen::VectorXd::Zero(1);
            error_dot_ = Eigen::VectorXd::Zero(1);
            error_int_ = Eigen::VectorXd::Zero(1);
        }
    }

private:
    double a_;
    double b_;
    double c_pos_;
    double c_neg_;

    double alpha;
    double beta;
    double gamma1;
    double gamma2;

    double kp_;    
    double ki_; 
    double kd_;
};

#endif // DC_MOTOR_HPP