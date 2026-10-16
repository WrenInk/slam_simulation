#!/usr/bin/env python3
"""Port the official plugin's Euler(0, zenith - pi/2, azimuth) * unit X."""
from pathlib import Path
import numpy as np
root = Path(__file__).resolve().parents[1]
rows = np.loadtxt(root/'third_party/livox_laser_simulation/scan_mode/mid360.csv',
                  delimiter=',', skiprows=1)
azimuth = np.deg2rad(rows[:, 1])
elevation = np.deg2rad(90. - rows[:, 2])
directions = np.column_stack((np.cos(elevation)*np.cos(azimuth),
                             np.cos(elevation)*np.sin(azimuth), np.sin(elevation)))
output = root/'src/nav_stack/scan_patterns/mid360_directions.npy'
output.parent.mkdir(parents=True, exist_ok=True)
np.save(output, directions.astype(np.float32))
print(len(rows), 'directions saved to', output)
