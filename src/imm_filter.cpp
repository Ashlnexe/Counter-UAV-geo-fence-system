#define _USE_MATH_DEFINES
#include "imm_filter.h"
#include <cmath>
#include <iostream>

IMMFilter::IMMFilter(double noise_cv, double noise_ca, double meas_noise)
    : noise_cv_(noise_cv), noise_ca_(noise_ca) {
    // 1. Initialize 6D State Vectors [x, y, vx, vy, ax, ay]
    x_cv = Eigen::VectorXd::Zero(6);
    x_ca = Eigen::VectorXd::Zero(6);
    x_out = Eigen::VectorXd::Zero(6);
    
    P_cv = Eigen::MatrixXd::Identity(6, 6) * 500.0; // High initial uncertainty
    P_ca = Eigen::MatrixXd::Identity(6, 6) * 500.0;
    P_out = P_cv;

    // 2. Initialize Transition Matrices to Identity (dt-dependent parts set by updateMatrices)
    F_cv = Eigen::MatrixXd::Identity(6, 6);
    F_ca = Eigen::MatrixXd::Identity(6, 6);

    // 3. Process noise initialized to zero (set by updateMatrices on first call)
    Q_cv = Eigen::MatrixXd::Zero(6, 6);
    Q_ca = Eigen::MatrixXd::Zero(6, 6);

    // 4. Measurement Matrix (H) - We only observe x and y via GPS
    H = Eigen::MatrixXd::Zero(2, 6);
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    R = Eigen::MatrixXd::Identity(2, 2) * (meas_noise * meas_noise);

    // 5. Markov Transition Matrix & Initial Probabilities
    Pi = Eigen::MatrixXd(2, 2);
    Pi << 0.95, 0.05,  // P(CV->CV), P(CV->CA)
          0.10, 0.90;  // P(CA->CV), P(CA->CA)
          
    mu = Eigen::VectorXd(2);
    mu << 0.8, 0.2; // Start by assuming cruising
}

void IMMFilter::updateMatrices(double dt) {
    // 1. Update Transition Matrices (F)
    F_cv = Eigen::MatrixXd::Identity(6, 6);
    F_cv(0, 2) = dt; F_cv(1, 3) = dt; 
    // CV zeroes out acceleration influence
    
    F_ca = Eigen::MatrixXd::Identity(6, 6);
    F_ca(0, 2) = dt; F_ca(1, 3) = dt;
    F_ca(0, 4) = 0.5 * dt * dt; F_ca(1, 5) = 0.5 * dt * dt;
    F_ca(2, 4) = dt; F_ca(3, 5) = dt;

    // 2. Update Process Noise (Q)
    double dt2 = dt*dt;
    double dt3 = dt2*dt;
    double dt4 = dt3*dt;
    
    Eigen::MatrixXd Q_base = Eigen::MatrixXd::Zero(6, 6);
    Q_base(0,0) = dt4/4; Q_base(0,2) = dt3/2; Q_base(0,4) = dt2/2;
    Q_base(1,1) = dt4/4; Q_base(1,3) = dt3/2; Q_base(1,5) = dt2/2;
    Q_base(2,0) = dt3/2; Q_base(2,2) = dt2;   Q_base(2,4) = dt;
    Q_base(3,1) = dt3/2; Q_base(3,3) = dt2;   Q_base(3,5) = dt;
    Q_base(4,0) = dt2/2; Q_base(4,2) = dt;    Q_base(4,4) = 1.0;
    Q_base(5,1) = dt2/2; Q_base(5,3) = dt;    Q_base(5,5) = 1.0;
    
    Q_cv = Q_base * noise_cv_; 
    Q_ca = Q_base * noise_ca_; 
}

double IMMFilter::calculateLikelihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& S) {
    double det = S.determinant();
    if (det <= 0.0) return 1e-300; // Guard against singular S
    Eigen::MatrixXd invS = S.inverse();
    double norm_factor = 1.0 / std::sqrt(std::pow(2 * M_PI, 2) * det);
    double exponent = -0.5 * (y.transpose() * invS * y)(0,0);
    return std::max(norm_factor * std::exp(exponent), 1e-300);
}

void IMMFilter::predict(double dt) {
    if (dt <= 0.0) return;
    
    // Rebuild F and Q for this time step
    updateMatrices(dt);

    // Predict both models forward without measurement update
    x_cv = F_cv * x_cv;
    P_cv = F_cv * P_cv * F_cv.transpose() + Q_cv;

    x_ca = F_ca * x_ca;
    P_ca = F_ca * P_ca * F_ca.transpose() + Q_ca;

    // Combine outputs (probabilities unchanged — no measurement to evaluate)
    x_out = x_cv * mu(0) + x_ca * mu(1);
    P_out = mu(0) * (P_cv + (x_cv - x_out)*(x_cv - x_out).transpose()) + 
            mu(1) * (P_ca + (x_ca - x_out)*(x_ca - x_out).transpose());
}

void IMMFilter::update(double z_x, double z_y, double dt) {
    if (dt <= 0.0) dt = 1e-6; // Guard against zero dt
    
    // Rebuild F and Q for this time step
    updateMatrices(dt);

    Eigen::VectorXd z(2);
    z << z_x, z_y;

    // --- STEP 1: INTERACTION / MIXING ---
    Eigen::VectorXd c = Pi.transpose() * mu;
    
    Eigen::MatrixXd mu_mix(2, 2);
    for (int i=0; i<2; ++i) {
        for (int j=0; j<2; ++j) {
            mu_mix(i,j) = Pi(i,j) * mu(i) / c(j);
        }
    }

    Eigen::VectorXd x0_cv = x_cv * mu_mix(0,0) + x_ca * mu_mix(1,0);
    Eigen::VectorXd x0_ca = x_cv * mu_mix(0,1) + x_ca * mu_mix(1,1);

    auto mixCovariance = [](const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, 
                            const Eigen::MatrixXd& P1, const Eigen::MatrixXd& P2, 
                            double w1, double w2) {
        Eigen::VectorXd diff1 = x1 - x0;
        Eigen::VectorXd diff2 = x2 - x0;
        return w1 * (P1 + diff1 * diff1.transpose()) + w2 * (P2 + diff2 * diff2.transpose());
    };

    Eigen::MatrixXd P0_cv = mixCovariance(x0_cv, x_cv, x_ca, P_cv, P_ca, mu_mix(0,0), mu_mix(1,0));
    Eigen::MatrixXd P0_ca = mixCovariance(x0_ca, x_cv, x_ca, P_cv, P_ca, mu_mix(0,1), mu_mix(1,1));

    // --- STEP 2: MODE-MATCHED FILTERING (Predict & Update) ---
    // CV Predict
    Eigen::VectorXd x_pred_cv = F_cv * x0_cv;
    Eigen::MatrixXd P_pred_cv = F_cv * P0_cv * F_cv.transpose() + Q_cv;
    // CA Predict
    Eigen::VectorXd x_pred_ca = F_ca * x0_ca;
    Eigen::MatrixXd P_pred_ca = F_ca * P0_ca * F_ca.transpose() + Q_ca;

    // Measurement Residuals
    Eigen::VectorXd y_cv = z - (H * x_pred_cv);
    Eigen::MatrixXd S_cv = H * P_pred_cv * H.transpose() + R;
    Eigen::VectorXd y_ca = z - (H * x_pred_ca);
    Eigen::MatrixXd S_ca = H * P_pred_ca * H.transpose() + R;

    // Kalman Gains & Updates
    Eigen::MatrixXd K_cv = P_pred_cv * H.transpose() * S_cv.inverse();
    x_cv = x_pred_cv + K_cv * y_cv;
    P_cv = (Eigen::MatrixXd::Identity(6, 6) - K_cv * H) * P_pred_cv;

    Eigen::MatrixXd K_ca = P_pred_ca * H.transpose() * S_ca.inverse();
    x_ca = x_pred_ca + K_ca * y_ca;
    P_ca = (Eigen::MatrixXd::Identity(6, 6) - K_ca * H) * P_pred_ca;

    // --- STEP 3: MODE PROBABILITY UPDATE ---
    Eigen::VectorXd Lambda(2);
    Lambda(0) = calculateLikelihood(y_cv, S_cv);
    Lambda(1) = calculateLikelihood(y_ca, S_ca);

    mu(0) = Lambda(0) * c(0);
    mu(1) = Lambda(1) * c(1);
    double mu_sum = mu(0) + mu(1);
    if (mu_sum > 0.0) {
        mu /= mu_sum; // Normalize to ensure probabilities sum to 1.0
    } else {
        mu << 0.5, 0.5; // Fallback if both likelihoods collapsed
    }

    // --- STEP 4: COMBINATION ---
    x_out = x_cv * mu(0) + x_ca * mu(1);
    P_out = mu(0) * (P_cv + (x_cv - x_out)*(x_cv - x_out).transpose()) + 
            mu(1) * (P_ca + (x_ca - x_out)*(x_ca - x_out).transpose());
}
