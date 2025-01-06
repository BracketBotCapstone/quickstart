import glob
import os

import matplotlib.pyplot as plt
import pandas as pd

# Find the latest CSV file
csv_files = glob.glob('PID_test_results_*.csv')
latest_file = max(csv_files, key=os.path.getctime)

# Read the CSV file
df = pd.read_csv(latest_file)

# Convert Time to seconds and subtract the initial time
df['Time'] = pd.to_datetime(df['Time'], unit='s')
t0 = df['Time'].iloc[0]
df['Time'] = (df['Time'] - t0).dt.total_seconds()

# Plot Velocity, Smoothed Velocity, and Desired_Velocity against Time
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(df['Time'], df['Velocity'], label='Actual Velocity', alpha=0.4)
plt.plot(df['Time'], df['Smoothed_Velocity'], label='Smoothed Velocity')
plt.plot(df['Time'], df['Desired_Velocity'], label='Desired Velocity')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity')
plt.title('Actual, Smoothed, and Desired Velocity over Time')
plt.legend()
plt.grid(True)

# Plot Angle, Raw Angle, and Desired_Angle against Time
plt.subplot(2, 1, 2)

# Smooth the raw angle with alpha = 0.9
alpha = 0.9
smoothed_raw_angle = df['Raw_Angle'].ewm(alpha=1-alpha).mean()

plt.plot(df['Time'], smoothed_raw_angle, label='Smoothed Raw Angle', alpha=0.4)
plt.plot(df['Time'], df['Angle'], label='Actual Angle')
plt.plot(df['Time'], df['Desired_Angle'], label='Desired Angle')
plt.xlabel('Time (seconds)')
plt.ylabel('Angle')
plt.title('Raw, Smoothed Raw, Actual, and Desired Angle over Time')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.savefig('plotted_results.png')