// =============================================================================
// kalman6d.cpp — C++ Core for Counter-UAV Tracking Engine
// =============================================================================

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <vector>
#include <deque>
#include <tuple>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <set>
#include <string>

#include "imm_filter.h"

namespace py = pybind11;

// =============================================================================
// KalmanFilter6D — Legacy 6D Constant-Velocity filter (retained for compat)
// =============================================================================

struct KFState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Vector<double, 6> x;
    Eigen::Matrix<double, 6, 6> P;
};

struct Measurement {
    double timestamp;
    double easting;
    double northing;
    double alt;
    double noise_std_m;
};

class KalmanFilter6D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector<double, 6> x;
    Eigen::Matrix<double, 6, 6> P;
    Eigen::Matrix<double, 3, 6> H;
    Eigen::Matrix3d R;
    double sa2;
    double last_time;

    std::deque<KFState, Eigen::aligned_allocator<KFState>> state_history;
    std::deque<Measurement> meas_history;

    KalmanFilter6D(double init_easting, double init_northing, double init_alt = 0.0)
        : sa2(0.3 * 0.3), last_time(0.0)
    {
        x << init_easting, init_northing, init_alt, 0.0, 0.0, 0.0;
        P = Eigen::Matrix<double, 6, 6>::Identity() * 100.0;
        H.setZero();
        H(0, 0) = 1.0; H(1, 1) = 1.0; H(2, 2) = 1.0;
        R = Eigen::Matrix3d::Identity() * (2.0 * 2.0);
    }

    void predict(double dt) {
        if (dt <= 0.0) return;
        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F(0, 3) = dt; F(1, 4) = dt; F(2, 5) = dt;
        double q_pos = 0.25 * dt * dt * dt * dt * sa2;
        double q_cov = 0.5  * dt * dt * dt * sa2;
        double q_vel = dt * dt * sa2;
        Eigen::Matrix<double, 6, 6> Q; Q.setZero();
        Q(0, 0) = q_pos; Q(1, 1) = q_pos; Q(2, 2) = q_pos;
        Q(3, 3) = q_vel; Q(4, 4) = q_vel; Q(5, 5) = q_vel;
        Q(0, 3) = q_cov; Q(3, 0) = q_cov; Q(1, 4) = q_cov; Q(4, 1) = q_cov; Q(2, 5) = q_cov; Q(5, 2) = q_cov;

        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    double compute_mahalanobis_distance(
        double easting_meas, double northing_meas, double alt_meas,
        double noise_std_m
    ) const {
        Eigen::Matrix3d R_local = Eigen::Matrix3d::Identity() * (noise_std_m * noise_std_m);
        Eigen::Vector3d z(easting_meas, northing_meas, alt_meas);
        Eigen::Vector3d y = z - H * x;
        Eigen::Matrix3d S = H * P * H.transpose() + R_local;
        return std::sqrt(y.transpose() * S.inverse() * y);
    }

    void update(double easting_meas, double northing_meas, double alt_meas, double noise_std_m = 2.0) {
        R = Eigen::Matrix3d::Identity() * (noise_std_m * noise_std_m);
        Eigen::Vector3d z(easting_meas, northing_meas, alt_meas);
        Eigen::Vector3d y = z - H * x;
        Eigen::Matrix3d S = H * P * H.transpose() + R;
        Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();
        x = x + K * y;
        Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
        P = I_KH * P * I_KH.transpose() + K * R * K.transpose();
    }

    void step_oosm(double meas_time, double easting, double northing, double alt, double noise_std_m, double current_time) {
        if (state_history.empty()) {
            last_time = current_time;
            state_history.push_back({last_time, x, P});
        }
        
        meas_history.push_back({meas_time, easting, northing, alt, noise_std_m});
        std::sort(meas_history.begin(), meas_history.end(), 
            [](const Measurement& a, const Measurement& b) { return a.timestamp < b.timestamp; });
            
        KFState rewind_state = state_history.front();
        for (auto it = state_history.rbegin(); it != state_history.rend(); ++it) {
            if (it->timestamp < meas_time) {
                rewind_state = *it;
                break;
            }
        }
        
        x = rewind_state.x;
        P = rewind_state.P;
        double t = rewind_state.timestamp;
        
        for (const auto& m : meas_history) {
            if (m.timestamp > t) {
                predict(m.timestamp - t);
                update(m.easting, m.northing, m.alt, m.noise_std_m);
                t = m.timestamp;
            }
        }
        
        if (current_time > t) {
            predict(current_time - t);
            t = current_time;
        }
        
        state_history.push_back({t, x, P});
        last_time = t;
        
        while (!state_history.empty() && state_history.front().timestamp < current_time - 15.0) {
            state_history.pop_front();
        }
        while (!meas_history.empty() && meas_history.front().timestamp < current_time - 15.0) {
            meas_history.pop_front();
        }
    }

    double compute_mahalanobis_oosm(double meas_e, double meas_n, double meas_alt, double meas_time, double noise_std_m) const {
        if (state_history.empty()) return compute_mahalanobis_distance(meas_e, meas_n, meas_alt, noise_std_m);
        
        KFState rewind_state = state_history.front();
        for (auto it = state_history.rbegin(); it != state_history.rend(); ++it) {
            if (it->timestamp <= meas_time) {
                rewind_state = *it;
                break;
            }
        }
        
        KalmanFilter6D temp_kf(rewind_state.x(0), rewind_state.x(1), rewind_state.x(2));
        temp_kf.x = rewind_state.x;
        temp_kf.P = rewind_state.P;
        double t = rewind_state.timestamp;
        
        for (const auto& m : meas_history) {
            if (m.timestamp > t && m.timestamp < meas_time) {
                temp_kf.predict(m.timestamp - t);
                temp_kf.update(m.easting, m.northing, m.alt, m.noise_std_m);
                t = m.timestamp;
            }
        }
        if (meas_time > t) {
            temp_kf.predict(meas_time - t);
        }
        return temp_kf.compute_mahalanobis_distance(meas_e, meas_n, meas_alt, noise_std_m);
    }

    std::tuple<double, double, double> step(
        double easting_meas, double northing_meas, double alt_meas,
        double dt, double noise_std_m = 2.0)
    {
        predict(dt);
        update(easting_meas, northing_meas, alt_meas, noise_std_m);
        return std::make_tuple(x(0), x(1), x(2));
    }

    std::vector<std::tuple<double, double, double>> project_future(int steps, double dt) const {
        Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
        F(0, 3) = dt; F(1, 4) = dt; F(2, 5) = dt;
        std::vector<std::tuple<double, double, double>> path;
        path.reserve(steps);
        Eigen::Vector<double, 6> x_proj = x;
        for (int i = 0; i < steps; ++i) {
            x_proj = F * x_proj;
            path.emplace_back(x_proj(0), x_proj(1), x_proj(2));
        }
        return path;
    }

    double estimated_speed_mps() const { return std::sqrt(x(3)*x(3) + x(4)*x(4) + x(5)*x(5)); }
    double estimated_vertical_speed_mps() const { return x(5); }
    std::tuple<double, double, double> position_variances() const { return std::make_tuple(P(0,0), P(1,1), P(2,2)); }
    Eigen::Vector<double, 6> get_x() const { return x; }
    Eigen::Matrix<double, 6, 6> get_P() const { return P; }
};

// =============================================================================
// GeofenceEngine — 3D cylinder zone checking with Schmitt trigger hysteresis
// =============================================================================

struct CylinderZone {
    std::string name;
    double center_easting;
    double center_northing;
    double radius;
    double alt_floor;
    double alt_ceiling;
};

class GeofenceEngine {
private:
    std::vector<CylinderZone> zones;
    std::map<std::string, std::set<std::string>> active_breaches;
    const double SCHMITT_BUFFER_M = 20.0;

public:
    void add_zone(std::string name, double e, double n, double r, double floor, double ceil) {
        zones.push_back({name, e, n, r, floor, ceil});
    }

    std::vector<std::string> check_breaches(std::string drone_id, double easting, double northing, double alt, Eigen::Matrix<double, 6, 6> P) {
        std::vector<std::string> current_breaches;
        
        double var_z = std::max(P(2,2), 1.0); 
        double z_bound = 3.0 * std::sqrt(var_z);

        Eigen::Matrix2d Pxy = P.block<2,2>(0,0);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(Pxy);
        
        double lambda_max = std::max(eigensolver.eigenvalues().maxCoeff(), 1.0);
        double uncertainty_radius = 3.0 * std::sqrt(lambda_max);

        for (const auto& zone : zones) {
            bool currently_breached = active_breaches[drone_id].count(zone.name) > 0;
            
            double effective_radius = zone.radius + uncertainty_radius;
            if (currently_breached) {
                effective_radius += SCHMITT_BUFFER_M;
            }

            bool alt_breach = (alt + z_bound >= zone.alt_floor) && (alt - z_bound <= zone.alt_ceiling);
            
            double dx = easting - zone.center_easting;
            double dy = northing - zone.center_northing;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (alt_breach && dist <= effective_radius) {
                current_breaches.push_back(zone.name);
            }
        }
        
        active_breaches[drone_id] = std::set<std::string>(current_breaches.begin(), current_breaches.end());
        return current_breaches;
    }
};

// =============================================================================
// Pybind11 Module — Exposes IMMFilter, KalmanFilter6D, GeofenceEngine
// =============================================================================

PYBIND11_MODULE(counter_uav_core, m) {
    m.doc() = "C++ Core for Counter-UAV Tracking Engine";

    // --- IMM Filter ---
    py::class_<IMMFilter>(m, "IMMFilter")
        .def(py::init<double, double, double>(), 
             py::arg("noise_cv"), py::arg("noise_ca"), py::arg("meas_noise"))
        .def("predict", &IMMFilter::predict, py::arg("dt"))
        .def("update", &IMMFilter::update, py::arg("z_x"), py::arg("z_y"), py::arg("dt"))
        .def_readonly("x_out", &IMMFilter::x_out)
        .def_readonly("P_out", &IMMFilter::P_out)
        .def_readonly("S_out", &IMMFilter::S_out)
        .def_readonly("mu", &IMMFilter::mu);

    // --- GeofenceEngine ---
    py::class_<GeofenceEngine>(m, "GeofenceEngine")
        .def(py::init<>())
        .def("add_zone", &GeofenceEngine::add_zone)
        .def("check_breaches", &GeofenceEngine::check_breaches);

    // --- Legacy KalmanFilter6D ---
    py::class_<KalmanFilter6D>(m, "KalmanFilter6D")
        .def(py::init<double, double, double>(),
             py::arg("init_easting"), py::arg("init_northing"), py::arg("init_alt") = 0.0)
        .def("predict", &KalmanFilter6D::predict, py::arg("dt"))
        .def("compute_mahalanobis_distance", &KalmanFilter6D::compute_mahalanobis_distance)
        .def("update", &KalmanFilter6D::update, py::arg("easting_meas"), py::arg("northing_meas"), py::arg("alt_meas"), py::arg("noise_std_m") = 2.0)
        .def("step", &KalmanFilter6D::step, py::arg("easting_meas"), py::arg("northing_meas"), py::arg("alt_meas"), py::arg("dt"), py::arg("noise_std_m") = 2.0)
        .def("step_oosm", &KalmanFilter6D::step_oosm, py::arg("meas_time"), py::arg("easting"), py::arg("northing"), py::arg("alt"), py::arg("noise_std_m"), py::arg("current_time"))
        .def("compute_mahalanobis_oosm", &KalmanFilter6D::compute_mahalanobis_oosm, py::arg("meas_e"), py::arg("meas_n"), py::arg("meas_alt"), py::arg("meas_time"), py::arg("noise_std_m"))
        .def("project_future", &KalmanFilter6D::project_future, py::arg("steps"), py::arg("dt"))
        .def_property_readonly("estimated_speed_mps", &KalmanFilter6D::estimated_speed_mps)
        .def_property_readonly("estimated_vertical_speed_mps", &KalmanFilter6D::estimated_vertical_speed_mps)
        .def_property_readonly("position_variances", &KalmanFilter6D::position_variances)
        .def("get_P", &KalmanFilter6D::get_P)
        .def("get_x", &KalmanFilter6D::get_x)
        .def_property("x",
            [](const KalmanFilter6D &kf) { return kf.x; },
            [](KalmanFilter6D &kf, const Eigen::Vector<double, 6> &val) { kf.x = val; })
        .def_property("P",
            [](const KalmanFilter6D &kf) { return kf.P; },
            [](KalmanFilter6D &kf, const Eigen::Matrix<double, 6, 6> &val) { kf.P = val; });
}
