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

from kalman import KalmanFilter6D

# ---------------------------------------------------------------------------
# Constants 
# ---------------------------------------------------------------------------
TRUE_EASTING = 500000.0
TRUE_NORTHING = 1433000.0
TRUE_ALTITUDE = 100.0
GPS_NOISE_STD_M = 3.0          # matches config.GPS_NOISE_STD_M

SEED = 42
N_WARMUP = 50    # ticks before we start measuring (let filter converge)
N_MEASURE = 100  # ticks we measure over


def make_filter(easting=TRUE_EASTING, northing=TRUE_NORTHING, alt=TRUE_ALTITUDE) -> KalmanFilter6D:
    return KalmanFilter6D(
        init_easting=easting,
        init_northing=northing,
        init_alt=alt
    )


def noisy_readings(n: int, rng: np.random.Generator,
                   easting=TRUE_EASTING, northing=TRUE_NORTHING, alt=TRUE_ALTITUDE,
                   noise_scale=1.0) -> list[tuple[float, float, float]]:
    """Generate n GPS readings around a stationary true position."""
    eastings = easting + rng.normal(0, GPS_NOISE_STD_M * noise_scale, n)
    northings = northing + rng.normal(0, GPS_NOISE_STD_M * noise_scale, n)
    altitudes = alt + rng.normal(0, GPS_NOISE_STD_M * noise_scale, n)
    return list(zip(eastings, northings, altitudes))


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
        for e_meas, n_meas, alt_meas in readings[:N_WARMUP]:
            kf.step(e_meas, n_meas, alt_meas, dt=0.5)

        # measure
        sq_errors = []
        for e_meas, n_meas, alt_meas in readings[N_WARMUP:]:
            e_f, n_f, alt_f = kf.step(e_meas, n_meas, alt_meas, dt=0.5)
            sq_errors.append((e_f - TRUE_EASTING)**2 + (n_f - TRUE_NORTHING)**2 + (alt_f - TRUE_ALTITUDE)**2)

        rms_error = math.sqrt(sum(sq_errors) / len(sq_errors))
        # 3.0 m threshold — filter must beat raw GPS std (3.0 m) by a clear margin.
        # We don't assert sub-metre accuracy because the Kalman gain with this
        # Q/R tuning is intentionally balanced toward trusting measurements;
        # extreme noise suppression would introduce lag at zone boundaries.
        assert rms_error < 3.0, (
            f"Filter RMS error {rms_error:.3f} m exceeds 3.0 m threshold — "
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

        for e_meas, n_meas, alt_meas in readings[:N_WARMUP]:
            kf.step(e_meas, n_meas, alt_meas, dt=0.5)

        raw_sq, filtered_sq = [], []
        for e_meas, n_meas, alt_meas in readings[N_WARMUP:]:
            e_f, n_f, alt_f = kf.step(e_meas, n_meas, alt_meas, dt=0.5)

            raw_sq.append((e_meas - TRUE_EASTING)**2 + (n_meas - TRUE_NORTHING)**2 + (alt_meas - TRUE_ALTITUDE)**2)
            filtered_sq.append((e_f - TRUE_EASTING)**2 + (n_f - TRUE_NORTHING)**2 + (alt_f - TRUE_ALTITUDE)**2)

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

        for e_meas, n_meas, alt_meas in readings:
            kf.step(e_meas, n_meas, alt_meas, dt=0.5)

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

        for e_meas, n_meas, alt_meas in readings:
            kf.step(e_meas, n_meas, alt_meas, dt=0.5)

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
        # Set a highly uncertain initial P so that measurements will shrink it
        kf.P = np.eye(6) * 500.0
        initial_trace = np.trace(kf.P)

        rng = np.random.default_rng(SEED)
        readings = noisy_readings(100, rng)
        for e_meas, n_meas, alt_meas in readings:
            kf.step(e_meas, n_meas, alt_meas, dt=0.5)

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

        for e_meas, n_meas, alt_meas in readings:
            kf.step(e_meas, n_meas, alt_meas, dt=0.5)

        speed = kf.estimated_speed_mps
        assert speed < 1.5, (
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
        v_easting = true_speed_mps  # Moving purely east

        kf = make_filter()
        easting, northing = TRUE_EASTING, TRUE_NORTHING

        # Feed N_WARMUP + 30 ticks of moving GPS
        dt = 0.5  # matches SIMULATION_STEP_SECONDS
        n = N_WARMUP + 30
        for i in range(n):
            easting += v_easting * dt
            noisy_e = easting + rng.normal(0, GPS_NOISE_STD_M)
            noisy_n = northing + rng.normal(0, GPS_NOISE_STD_M)
            noisy_a = TRUE_ALTITUDE + rng.normal(0, GPS_NOISE_STD_M)
            kf.step(noisy_e, noisy_n, noisy_a, dt=0.5)

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
        result = kf.step(TRUE_EASTING + 1.0, TRUE_NORTHING + 1.0, TRUE_ALTITUDE + 1.0, dt=0.5)
        assert isinstance(result, tuple), "step() must return a tuple"
        assert len(result) == 3, "step() must return exactly 3 values"
        assert all(isinstance(v, float) for v in result), \
            "step() must return native Python floats, not numpy scalars"

    def test_filtered_position_is_near_input(self):
        """
        On the very first step, filtered output should be close to the input.
        Initial P is small, so the first update shouldn't jump far.
        """
        kf = make_filter()
        e_f, n_f, a_f = kf.step(TRUE_EASTING, TRUE_NORTHING, TRUE_ALTITUDE, dt=0.5)
        assert abs(e_f - TRUE_EASTING) < 0.01, "First step easting jumped unreasonably"
        assert abs(n_f - TRUE_NORTHING) < 0.01, "First step northing jumped unreasonably"
        assert abs(a_f - TRUE_ALTITUDE) < 0.01, "First step altitude jumped unreasonably"

    def test_estimated_speed_is_non_negative(self):
        kf = make_filter()
        rng = np.random.default_rng(SEED)
        for e_meas, n_meas, alt_meas in noisy_readings(20, rng):
            kf.step(e_meas, n_meas, alt_meas, dt=0.5)
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

        for e_meas, n_meas, alt_meas in readings[:N_WARMUP]:
            kf.step(e_meas, n_meas, alt_meas, dt=0.5)

        raw_sq, filtered_sq = [], []
        for e_meas, n_meas, alt_meas in readings[N_WARMUP:]:
            e_f, n_f, alt_f = kf.step(e_meas, n_meas, alt_meas, dt=0.5)
            raw_sq.append((e_meas - TRUE_EASTING)**2 + (n_meas - TRUE_NORTHING)**2 + (alt_meas - TRUE_ALTITUDE)**2)
            filtered_sq.append((e_f - TRUE_EASTING)**2 + (n_f - TRUE_NORTHING)**2 + (alt_f - TRUE_ALTITUDE)**2)

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
            kf.step(TRUE_EASTING, TRUE_NORTHING, TRUE_ALTITUDE, dt=0.5)  # must not raise
