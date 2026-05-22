"""
setup.py — Builds counter_uav_core C++ extension module.

Usage:
    python setup.py build_ext --inplace

The Eigen3 headers are fetched automatically if not already present.
"""

import os
import sys
import zipfile
import urllib.request
import shutil
from setuptools import setup, Extension

# ---------------------------------------------------------------------------
# 1. Auto-fetch Eigen3 headers if missing
# ---------------------------------------------------------------------------
EIGEN_VERSION = "3.4.0"
EIGEN_DIR = os.path.join(os.path.dirname(__file__), "extern", "eigen")
EIGEN_INCLUDE = os.path.join(EIGEN_DIR, f"eigen-{EIGEN_VERSION}")

if not os.path.isdir(EIGEN_INCLUDE):
    print(f"[setup.py] Downloading Eigen {EIGEN_VERSION} headers...")
    url = f"https://gitlab.com/libeigen/eigen/-/archive/{EIGEN_VERSION}/eigen-{EIGEN_VERSION}.zip"
    zip_path = os.path.join(os.path.dirname(__file__), "eigen.zip")

    urllib.request.urlretrieve(url, zip_path)

    os.makedirs(EIGEN_DIR, exist_ok=True)
    with zipfile.ZipFile(zip_path, "r") as zf:
        zf.extractall(EIGEN_DIR)
    os.remove(zip_path)
    print(f"[setup.py] Eigen headers extracted to {EIGEN_INCLUDE}")

# ---------------------------------------------------------------------------
# 2. pybind11 include paths
# ---------------------------------------------------------------------------
try:
    import pybind11
    pybind11_includes = pybind11.get_include()
except ImportError:
    print("ERROR: pybind11 is not installed. Run: pip install pybind11")
    sys.exit(1)

# ---------------------------------------------------------------------------
# 3. Define the C++ extension
# ---------------------------------------------------------------------------
ext = Extension(
    name="counter_uav_core",
    sources=[os.path.join("src", "kalman6d.cpp")],
    include_dirs=[
        pybind11_includes,
        EIGEN_INCLUDE,
    ],
    language="c++",
    extra_compile_args=(
        ["/std:c++17", "/O2", "/EHsc"]  # MSVC flags
        if sys.platform == "win32"
        else ["-std=c++17", "-O3", "-fPIC"]  # GCC/Clang flags
    ),
)

# ---------------------------------------------------------------------------
# 4. Build
# ---------------------------------------------------------------------------
setup(
    name="counter_uav_core",
    version="1.0.0",
    description="C++ core for Counter-UAV: 6D Kalman Filter with Eigen3",
    ext_modules=[ext],
)
