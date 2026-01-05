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



def trajectory_stateful(func):
    state = {
        "vx": 0.0, "vy": 0.0, "vz": 0.0,
        "px": 0.0, "py": 0.0, "pz": 0.0,
        "mean_ax": 0.0, "mean_ay": 0.0, "mean_az": 0.0,
        "count": 1
    }

    def wrapper(ax, ay, az, dt):
        nonlocal state

        ax -=  state["mean_ax"]
        ay -= state["mean_ay"]
        az -= state["mean_az"]

        px, py, pz, vx, vy, vz = func(
            ax, ay, az,
            state["vx"], state["vy"], state["vz"],
            state["px"], state["py"], state["pz"],
            dt
        )

        mean_ax, mean_ay, mean_az = state['mean_ax'], state["mean_ay"], state["mean_az"]

        mean_ax, mean_ay, mean_az = (ax + mean_ax * state["count"]) / state["count"] , (ay + mean_ay * state["count"]) / state["count"], (az + mean_az * state["count"]) / state["count"]


        state.update({
            "vx": vx, "vy": vy, "vz": vz,
            "px": px, "py": py, "pz": pz,
            "count": state["count"] + 1, 
        })

        return px, py, pz, vx, vy, vz

    # Optional reset hook (important for experiments)
    def reset():
        state.update({
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "px": 0.0, "py": 0.0, "pz": 0.0,
        })

    wrapper.reset = reset
    return wrapper


@trajectory_stateful
def estimate_trajectory_for_queue(
    ax, ay, az,
    prev_vx, prev_vy, prev_vz,
    prev_px, prev_py, prev_pz,
    dt
):
    vx = ax * dt + prev_vx
    vy = ay * dt + prev_vy
    vz = az * dt + prev_vz

    px = vx * dt + prev_px
    py = vy * dt + prev_py
    pz = vz * dt + prev_pz

    return px, py, pz, vx, vy, vz

