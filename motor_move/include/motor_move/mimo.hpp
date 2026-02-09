#ifndef MIMO_PID_HPP
#define MIMO_PID_HPP

#include <eigen3/Eigen/Dense>

class MIMO_PID {
    public:
        MIMO_PID(const Eigen::MatrixXd Kp, const Eigen::MatrixXd Ki, const Eigen::MatrixXd Kd);
        MIMO_PID();
        ~MIMO_PID();
        
        // New: compute with position for Derivative on Measurement
        Eigen::MatrixXd compute(const Eigen::MatrixXd error, const Eigen::MatrixXd position, const double dt);
        
        void set_Kp(const Eigen::MatrixXd Kp);
        void set_Ki(const Eigen::MatrixXd Ki);
        void set_Kd(const Eigen::MatrixXd Kd);
        
        void set_integral_limits(const Eigen::VectorXd& min_limits, const Eigen::VectorXd& max_limits);
        void set_integral_limits(double min_val, double max_val);
        void set_p_term_limits(double max_linear, double max_angular);
        void reset_integral();
        void reset();  // Reset all state for new goal
        
    private:
        Eigen::MatrixXd Kp;
        Eigen::MatrixXd Ki;
        Eigen::MatrixXd Kd;
        Eigen::MatrixXd integral;
        Eigen::MatrixXd position_prev;
        bool first_run = true;
        
        Eigen::VectorXd integral_min;
        Eigen::VectorXd integral_max;
        bool use_anti_windup = false;
        
        double p_max_linear = 0.4;
        double p_max_angular = 1.0;
        bool use_p_limits = true;
        
        void clamp_integral();
};

#endif // MIMO_PID_HPP
