import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid
from scipy.signal import butter, filtfilt

def detect_stationary(ax, ay, az, threshold=0.05):
    """
    Detect stationary periods based on acceleration magnitude.

    Returns
    -------
    stationary : boolean np.ndarray
        True where device is stationary
    """
    acc_mag = np.sqrt(ax**2 + ay**2 + az**2)
    return acc_mag < threshold

def apply_zupt(vx, vy, vz, stationary):
    """
    Enforce zero velocity during stationary intervals.
    """
    vx = vx.copy()
    vy = vy.copy()
    vz = vz.copy()

    vx[stationary] = 0.0
    vy[stationary] = 0.0
    vz[stationary] = 0.0

    return vx, vy, vz


def compute_vector_displacement(time, ax, ay, az):
    """
    Compute true displacement using vector integration.

    Parameters
    ----------
    time : np.ndarray
        Time vector (seconds)
    ax, ay, az : np.ndarray
        Linear acceleration components (m/s^2)

    Returns
    -------
    displacement_mag : float
        Net displacement from start (meters)
    position : tuple of np.ndarray
        (px, py, pz) position trajectory
    """

    # Sampling frequency
    dt = np.mean(np.diff(time))
    fs = 1.0 / dt
    print(f"Fs = {fs}")

    # ---- Bias removal ----
    ax = ax - np.mean(ax)
    ay = ay - np.mean(ay)
    az = az - np.mean(az)

    # ---- High-pass filtering ----
    ax = high_pass_filter(ax, fs)
    ay = high_pass_filter(ay, fs)
    az = high_pass_filter(az, fs)

    # ---- Integrate acceleration -> velocity ----
    vx = cumulative_trapezoid(ax, time, initial=0.0)
    vy = cumulative_trapezoid(ay, time, initial=0.0)
    vz = cumulative_trapezoid(az, time, initial=0.0)

    # ---- ZUPT detection ----
    stationary = detect_stationary(ax, ay, az)

    # ---- Apply ZUPT ----
    vx, vy, vz = apply_zupt(vx, vy, vz, stationary)

    # ---- Integrate velocity -> position ----
    px = cumulative_trapezoid(vx, time, initial=0.0)
    py = cumulative_trapezoid(vy, time, initial=0.0)
    pz = cumulative_trapezoid(vz, time, initial=0.0)

    # ---- Net displacement magnitude ----
    displacement_mag = np.sqrt(px[-1]**2 + py[-1]**2 + pz[-1]**2)

    return displacement_mag, (px, py, pz)


def remove_mean_bias(acc):
    """
    Remove constant bias by subtracting mean acceleration.
    This assumes the device is stationary at some point.
    """
    return acc - np.mean(acc)

def high_pass_filter(signal, fs, cutoff=0.1, order=2):
    """
    High-pass filter to remove low-frequency drift.

    fs     : sampling frequency (Hz)
    cutoff : cutoff frequency (Hz)
    """
    nyquist = 0.5 * fs
    norm_cutoff = cutoff / nyquist

    b, a = butter(order, norm_cutoff, btype='high')
    return filtfilt(b, a, signal)

def integrate(signal, time):
    """
    Integrate signal w.r.t. time using cumulative trapezoidal rule.
    """
    return cumulative_trapezoid(signal, time, initial=0.0)


def estimate_trajectory(time, ax, ay, az):
    """
    Full IMU dead-reckoning pipeline:
    acceleration -> velocity -> position
    """

    # Sampling frequency
    dt = np.mean(np.diff(time))
    fs = 1.0 / dt

    # ---- Bias correction ----
    ax = remove_mean_bias(ax)
    ay = remove_mean_bias(ay)
    az = remove_mean_bias(az)

    # ---- High-pass filtering ----
    ax = high_pass_filter(ax, fs)
    ay = high_pass_filter(ay, fs)
    az = high_pass_filter(az, fs)

    # ---- Integrate acceleration -> velocity ----
    vx = integrate(ax, time)
    vy = integrate(ay, time)
    vz = integrate(az, time)

    # ---- ZUPT detection ----
    stationary = detect_stationary(ax, ay, az)

    # ---- Apply ZUPT ----
    vx, vy, vz = apply_zupt(vx, vy, vz, stationary)

    # ---- Integrate velocity -> position ----
    px = integrate(vx, time)
    py = integrate(vy, time)
    pz = integrate(vz, time)

    return px, py, pz