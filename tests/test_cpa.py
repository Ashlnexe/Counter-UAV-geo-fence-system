import numpy as np
import pytest
from threat_engine import calculate_cpa, calculate_segment_cpa, calculate_polygon_cpa, point_to_segment_dist

def test_point_to_segment_dist():
    A = np.array([0.0, 0.0])
    B = np.array([10.0, 0.0])
    
    # Perpendicular to middle
    assert np.isclose(point_to_segment_dist(np.array([5.0, 5.0]), A, B), 5.0)
    
    # Outside A
    assert np.isclose(point_to_segment_dist(np.array([-5.0, 0.0]), A, B), 5.0)
    
    # Outside B
    assert np.isclose(point_to_segment_dist(np.array([15.0, 0.0]), A, B), 5.0)

def test_calculate_cpa_point():
    p_drone = np.array([0.0, 0.0])
    v_drone = np.array([10.0, 0.0])
    p_target = np.array([10.0, 5.0])
    
    # Drone flies along X axis. Target is at x=10, y=5.
    # CPA is at t=1.0, distance is 5.0.
    t, d = calculate_cpa(p_drone, v_drone, p_target)
    assert np.isclose(t, 1.0)
    assert np.isclose(d, 5.0)

def test_calculate_segment_cpa_intersection():
    p_drone = np.array([5.0, 10.0])
    v_drone = np.array([0.0, -10.0])
    A = np.array([0.0, 0.0])
    B = np.array([10.0, 0.0])
    
    # Drone flies down and hits segment AB exactly at midpoint (5.0, 0.0) at t=1.0
    t, d = calculate_segment_cpa(p_drone, v_drone, A, B)
    assert np.isclose(t, 1.0)
    assert np.isclose(d, 0.0)

def test_calculate_segment_cpa_miss_hits_vertex():
    p_drone = np.array([-5.0, 10.0])
    v_drone = np.array([0.0, -10.0])
    A = np.array([0.0, 0.0])
    B = np.array([10.0, 0.0])
    
    # Drone flies down at x=-5. Misses segment (0..10).
    # CPA should be at t=1.0, when drone is at (-5.0, 0.0). Closest point is A (0.0, 0.0), distance 5.0.
    t, d = calculate_segment_cpa(p_drone, v_drone, A, B)
    assert np.isclose(t, 1.0)
    assert np.isclose(d, 5.0)

def test_calculate_segment_cpa_parallel():
    p_drone = np.array([-10.0, 5.0])
    v_drone = np.array([10.0, 0.0])
    A = np.array([0.0, 0.0])
    B = np.array([10.0, 0.0])
    
    # Drone flies parallel. At t=0, dist to segment A is sqrt(100+25) = 11.18.
    # At t=1.0, drone is at (0, 5), dist to segment is 5.0.
    # The algorithm should return min_dist 5.0. 
    # Time could be 1.0 (when it passes A) or 2.0 (when it passes B).
    t, d = calculate_segment_cpa(p_drone, v_drone, A, B)
    assert np.isclose(d, 5.0)
    assert np.isclose(t, 1.0)  # Earliest time

def test_calculate_polygon_cpa_direct_hit():
    polygon = [
        np.array([0.0, 0.0]),
        np.array([10.0, 0.0]),
        np.array([10.0, 10.0]),
        np.array([0.0, 10.0])
    ]
    p_drone = np.array([-10.0, 5.0])
    v_drone = np.array([10.0, 0.0])
    
    # Hits the left edge (0,0 to 0,10) at (0, 5) at t=1.0.
    t, d = calculate_polygon_cpa(p_drone, v_drone, polygon)
    assert np.isclose(t, 1.0)
    assert np.isclose(d, 0.0)

def test_calculate_polygon_cpa_bypass():
    polygon = [
        np.array([0.0, 0.0]),
        np.array([10.0, 0.0]),
        np.array([10.0, 10.0]),
        np.array([0.0, 10.0])
    ]
    p_drone = np.array([-10.0, -5.0])
    v_drone = np.array([10.0, 0.0])
    
    # Flies completely underneath the polygon, along y=-5.
    # Closest it ever gets is when x=0 (passing first bottom vertex) and x=10 (passing second).
    # Minimum distance is 5.0, achieved at t=1.0.
    t, d = calculate_polygon_cpa(p_drone, v_drone, polygon)
    assert np.isclose(t, 1.0)
    assert np.isclose(d, 5.0)
