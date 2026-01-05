import pandas as pd
from calculate_methods import estimate_trajectory, compute_vector_displacement
from plot_methods import plot_3d

df = pd.read_csv('./acceleration_data/acceleration.csv')


time = df['Timestamp']
df['ts'] = df['Timestamp']
ax = df['X']
ay = df['Y']
az = df['Z']

ts = pd.to_datetime(df["ts"], format="%d-%b-%Y %H:%M:%S.%f")

df["ms_from_start"] = (
    (ts - ts.iloc[0]).dt.total_seconds()
).astype("int64")

time = df['ms_from_start']
px, py, pz = estimate_trajectory(time, ax, ay, az)
print(compute_vector_displacement(time, ax, ay, az))
plot_3d(px, py, pz)

