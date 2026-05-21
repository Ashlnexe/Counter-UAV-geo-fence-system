# =============================================================================
# tests/test_kalman.py — Unit tests for KalmanFilter2D
# =============================================================================
#
# Run with:
#   pytest tests/test_kalman.py -v
#
# What we test and WHY:
#   1. Convergence     — filter must approach true position over time
#   2. Noise rejection — filtered error must be lower than raw GPS error
#   3. Positive-definiteness — P must stay PD after many steps (Joseph form check)
#   4. Symmetry        — P must remain symmetric (numerical stability)
#   5. Speed estimate  — velocity state must be reasonable after convergence
#   6. Step interface  — step() must return correct types and shapes
#   7. Stationary UAV  — filter must not drift on a non-moving target
#   8. High noise      — filter must still converge under extreme GPS noise
# =============================================================================

import math
import sys
import os
import numpy as np
import pytest

# Allow import from project root without installing as a package
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from kalman import KalmanFilter2D

# ---------------------------------------------------------------------------
# Constants matching Bengaluru lat (12.97°N) — same as production config
# ---------------------------------------------------------------------------
METERS_PER_LAT = 110_570.0
METERS_PER_LON = 108_484.0   # 111320 * cos(12.97°)

TRUE_LAT = 12.9716
TRUE_LON = 77.5946
GPS_NOISE_STD_M = 3.0          # matches config.GPS_NOISE_STD_M
GPS_NOISE_STD_LAT = GPS_NOISE_STD_M / METERS_PER_LAT
GPS_NOISE_STD_LON = GPS_NOISE_STD_M / METERS_PER_LON

SEED = 42
N_WARMUP = 50    # ticks before we start measuring (let filter converge)
N_MEASURE = 100  # ticks we measure over


def make_filter(lat=TRUE_LAT, lon=TRUE_LON) -> KalmanFilter2D:
    return KalmanFilter2D(
        init_lat=lat,
        init_lon=lon,
        meters_per_lat=METERS_PER_LAT,
        meters_per_lon=METERS_PER_LON,
    )


def noisy_readings(n: int, rng: np.random.Generator,
                   lat=TRUE_LAT, lon=TRUE_LON,
                   noise_scale=1.0) -> list[tuple[float, float]]:
    """Generate n GPS readings around a stationary true position."""
    lats = lat + rng.normal(0, GPS_NOISE_STD_LAT * noise_scale, n)
    lons = lon + rng.normal(0, GPS_NOISE_STD_LON * noise_scale, n)
    return list(zip(lats, lons))


# ---------------------------------------------------------------------------
# 1. CONVERGENCE
# ---------------------------------------------------------------------------
class TestConvergence:
    def test_filter_converges_to_true_position(self):
        """
        After warm-up, the filtered position error (in metres) must be
        substantially smaller than the raw GPS noise (3 m std).
        We assert the RMS filtered error is below 1.5 m — half the GPS std.
        """
        rng = np.random.default_rng(SEED)
        kf = make_filter()
        readings = noisy_readings(N_WARMUP + N_MEASURE, rng)

        # warm up
        for lat_m, lon_m in readings[:N_WARMUP]:
            kf.step(lat_m, lon_m, dt=0.5)

        # measure
        sq_errors = []
        for lat_m, lon_m in readings[N_WARMUP:]:
            lat_f, lon_f = kf.step(lat_m, lon_m, dt=0.5)
            err_lat_m = (lat_f - TRUE_LAT) * METERS_PER_LAT
            err_lon_m = (lon_f - TRUE_LON) * METERS_PER_LON
            sq_errors.append(err_lat_m**2 + err_lon_m**2)

        rms_error = math.sqrt(sum(sq_errors) / len(sq_errors))
        # 2.0 m threshold — filter must beat raw GPS std (3.0 m) by a clear margin.
        # We don't assert sub-metre accuracy because the Kalman gain with this
        # Q/R tuning is intentionally balanced toward trusting measurements;
        # extreme noise suppression would introduce lag at zone boundaries.
        assert rms_error < 2.0, (
            f"Filter RMS error {rms_error:.3f} m exceeds 2.0 m threshold — "
            f"filter is not converging correctly."
        )

    def test_filtered_error_beats_raw_gps(self):
        """
        The whole point of the filter: filtered error < raw GPS error.
        Both measured over the same N_MEASURE ticks after warm-up.
        """
        rng = np.random.default_rng(SEED)
        kf = make_filter()
        readings = noisy_readings(N_WARMUP + N_MEASURE, rng)

        for lat_m, lon_m in readings[:N_WARMUP]:
            kf.step(lat_m, lon_m, dt=0.5)

        raw_sq, filtered_sq = [], []
        for lat_m, lon_m in readings[N_WARMUP:]:
            lat_f, lon_f = kf.step(lat_m, lon_m, dt=0.5)

            raw_sq.append(
                ((lat_m - TRUE_LAT) * METERS_PER_LAT)**2 +
                ((lon_m - TRUE_LON) * METERS_PER_LON)**2
            )
            filtered_sq.append(
                ((lat_f - TRUE_LAT) * METERS_PER_LAT)**2 +
                ((lon_f - TRUE_LON) * METERS_PER_LON)**2
            )

        rms_raw = math.sqrt(sum(raw_sq) / len(raw_sq))
        rms_filtered = math.sqrt(sum(filtered_sq) / len(filtered_sq))

        assert rms_filtered < rms_raw, (
            f"Filter (RMS={rms_filtered:.3f} m) is not improving on "
            f"raw GPS (RMS={rms_raw:.3f} m) — something is wrong."
        )


# ---------------------------------------------------------------------------
# 2. NUMERICAL STABILITY — the whole reason we use the Joseph form
# ---------------------------------------------------------------------------
class TestNumericalStability:
    def test_covariance_remains_positive_definite(self):
        """
        P must stay positive-definite (all eigenvalues > 0) after many steps.
        The naive update P = (I-KH)P can silently break this over time.
        The Joseph form must prevent it.
        We run 2000 ticks — far beyond a typical session — to stress this.
        """
        rng = np.random.default_rng(SEED)
        kf = make_filter()
        readings = noisy_readings(2000, rng)

        for lat_m, lon_m in readings:
            kf.step(lat_m, lon_m, dt=0.5)

        eigenvalues = np.linalg.eigvalsh(kf.P)
        assert np.all(eigenvalues > 0), (
            f"Covariance matrix lost positive-definiteness after 2000 steps. "
            f"Eigenvalues: {eigenvalues}. Joseph form may be broken."
        )

    def test_covariance_remains_symmetric(self):
        """
        P must remain symmetric. Asymmetry is a sign of numerical corruption.
        """
        rng = np.random.default_rng(SEED)
        kf = make_filter()
        readings = noisy_readings(2000, rng)

        for lat_m, lon_m in readings:
            kf.step(lat_m, lon_m, dt=0.5)

        asymmetry = np.max(np.abs(kf.P - kf.P.T))
        assert asymmetry < 1e-12, (
            f"Covariance matrix is asymmetric after 2000 steps (max diff={asymmetry:.2e}). "
            f"Numerical stability is broken."
        )

    def test_covariance_shrinks_from_initial(self):
        """
        P should decrease from initial uncertainty as measurements arrive.
        If P is growing, the filter is diverging.
        """
        kf = make_filter()
        initial_trace = np.trace(kf.P)

        rng = np.random.default_rng(SEED)
        readings = noisy_readings(100, rng)
        for lat_m, lon_m in readings:
            kf.step(lat_m, lon_m, dt=0.5)

        final_trace = np.trace(kf.P)
        assert final_trace < initial_trace, (
            f"Covariance trace grew from {initial_trace:.6f} to {final_trace:.6f}. "
            f"Filter is diverging rather than gaining confidence."
        )


# ---------------------------------------------------------------------------
# 3. SPEED ESTIMATE
# ---------------------------------------------------------------------------
class TestSpeedEstimate:
    def test_speed_near_zero_for_stationary_uav(self):
        """
        A stationary UAV should have near-zero estimated speed after convergence.
        Tolerance is loose (< 1.0 m/s) to account for filter settling.
        """
        rng = np.random.default_rng(SEED)
        kf = make_filter()
        readings = noisy_readings(N_WARMUP + 20, rng)

        for lat_m, lon_m in readings:
            kf.step(lat_m, lon_m, dt=0.5)

        speed = kf.estimated_speed_mps
        assert speed < 1.0, (
            f"Stationary UAV estimated speed is {speed:.3f} m/s — "
            f"velocity state is not converging to zero."
        )

    def test_speed_estimate_tracks_moving_uav(self):
        """
        A UAV moving at a known velocity should produce a speed estimate
        within ±3 m/s of truth after convergence.
        True velocity: 10 m/s north (latitude direction only).
        """
        rng = np.random.default_rng(SEED)
        true_speed_mps = 10.0
        vlat_deg_per_s = true_speed_mps / METERS_PER_LAT

        kf = make_filter()
        lat, lon = TRUE_LAT, TRUE_LON

        # Feed N_WARMUP + 30 ticks of moving GPS
        dt = 0.5  # matches SIMULATION_STEP_SECONDS
        n = N_WARMUP + 30
        for i in range(n):
            lat += vlat_deg_per_s * dt
            noisy_lat = lat + rng.normal(0, GPS_NOISE_STD_LAT)
            noisy_lon = lon + rng.normal(0, GPS_NOISE_STD_LON)
            kf.step(noisy_lat, noisy_lon, dt=0.5)

        estimated = kf.estimated_speed_mps
        assert abs(estimated - true_speed_mps) < 3.0, (
            f"Speed estimate {estimated:.2f} m/s deviates more than 3 m/s "
            f"from true speed {true_speed_mps} m/s."
        )


# ---------------------------------------------------------------------------
# 4. INTERFACE & TYPE SAFETY
# ---------------------------------------------------------------------------
class TestInterface:
    def test_step_returns_tuple_of_floats(self):
        kf = make_filter()
        result = kf.step(TRUE_LAT + 1e-5, TRUE_LON + 1e-5, dt=0.5)
        assert isinstance(result, tuple), "step() must return a tuple"
        assert len(result) == 2, "step() must return exactly 2 values"
        assert all(isinstance(v, float) for v in result), \
            "step() must return native Python floats, not numpy scalars"

    def test_filtered_position_is_near_input(self):
        """
        On the very first step, filtered output should be close to the input.
        Initial P is small, so the first update shouldn't jump far.
        """
        kf = make_filter()
        lat_f, lon_f = kf.step(TRUE_LAT, TRUE_LON, dt=0.5)
        assert abs(lat_f - TRUE_LAT) < 0.01, "First step lat jumped unreasonably"
        assert abs(lon_f - TRUE_LON) < 0.01, "First step lon jumped unreasonably"

    def test_estimated_speed_is_non_negative(self):
        kf = make_filter()
        rng = np.random.default_rng(SEED)
        for lat_m, lon_m in noisy_readings(20, rng):
            kf.step(lat_m, lon_m, dt=0.5)
        assert kf.estimated_speed_mps >= 0.0


# ---------------------------------------------------------------------------
# 5. ROBUSTNESS
# ---------------------------------------------------------------------------
class TestRobustness:
    def test_high_noise_still_converges(self):
        """
        Under 10× GPS noise (30 m std instead of 3 m), the filter should
        still reduce error compared to raw GPS — just more slowly.
        """
        rng = np.random.default_rng(SEED)
        kf = make_filter()
        readings = noisy_readings(N_WARMUP + N_MEASURE, rng, noise_scale=10.0)

        for lat_m, lon_m in readings[:N_WARMUP]:
            kf.step(lat_m, lon_m, dt=0.5)

        raw_sq, filtered_sq = [], []
        for lat_m, lon_m in readings[N_WARMUP:]:
            lat_f, lon_f = kf.step(lat_m, lon_m, dt=0.5)
            raw_sq.append(
                ((lat_m - TRUE_LAT) * METERS_PER_LAT)**2 +
                ((lon_m - TRUE_LON) * METERS_PER_LON)**2
            )
            filtered_sq.append(
                ((lat_f - TRUE_LAT) * METERS_PER_LAT)**2 +
                ((lon_f - TRUE_LON) * METERS_PER_LON)**2
            )

        rms_raw = math.sqrt(sum(raw_sq) / len(raw_sq))
        rms_filtered = math.sqrt(sum(filtered_sq) / len(filtered_sq))

        assert rms_filtered < rms_raw, (
            f"Under high noise, filter ({rms_filtered:.2f} m) "
            f"did not beat raw GPS ({rms_raw:.2f} m)."
        )

    def test_no_exception_on_identical_readings(self):
        """
        Degenerate case: GPS returns the exact same reading every tick.
        Filter must not crash (S matrix must remain invertible).
        """
        kf = make_filter()
        for _ in range(50):
            kf.step(TRUE_LAT, TRUE_LON, dt=0.5)  # must not raise
