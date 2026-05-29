#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class IMMFilter {
public:
    // Combined output state and covariance
    Eigen::VectorXd x_out; 
    Eigen::MatrixXd P_out; 
    Eigen::MatrixXd S_out;

    // Model probabilities [mu_cv, mu_ca]
    Eigen::VectorXd mu;    

    IMMFilter(double noise_cv, double noise_ca, double meas_noise);
    void predict(double dt);
    void update(double z_x, double z_y, double dt);

private:
    double noise_cv_;
    double noise_ca_;
    
    // Model 1: Constant Velocity (CV)
    Eigen::VectorXd x_cv;
    Eigen::MatrixXd P_cv;
    Eigen::MatrixXd F_cv;
    Eigen::MatrixXd Q_cv;

    // Model 2: Constant Acceleration (CA)
    Eigen::VectorXd x_ca;
    Eigen::MatrixXd P_ca;
    Eigen::MatrixXd F_ca;
    Eigen::MatrixXd Q_ca;

    // Shared Measurement Matrices
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;

    // Markov Transition Matrix
    Eigen::MatrixXd Pi;
    
    // Internal methods
    void updateMatrices(double dt);
    double calculateLikelihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& S);
};
