import numpy as np

# Camera FOV in degrees
HFOV = 87
VFOV = 58

# Convert FOV to radians
hfov_rad = np.deg2rad(HFOV)
vfov_rad = np.deg2rad(VFOV)

# Maximum depth range (in meters)
z_max = 5.0

# Compute spatial bounds
x_min = -z_max * np.tan(hfov_rad / 2)
x_max = z_max * np.tan(hfov_rad / 2)
y_min = -z_max * np.tan(vfov_rad / 2)
y_max = z_max * np.tan(vfov_rad / 2)

print(f"x_min: {x_min}, x_max: {x_max}")
print(f"y_min: {y_min}, y_max: {y_max}")
