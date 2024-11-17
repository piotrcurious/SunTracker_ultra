import math

# LUT generation parameters
azimuth_resolution = 1  # Degrees
elevation_resolution = 1  # Degrees (can be extended for 3D LUT)
fixed_point_scale = 1000  # Scale factor for fixed-point representation

# Generate LUT for azimuth (0 to 359 degrees) with fixed elevation
lut = []

# Iterate over azimuth angles
for azimuth in range(0, 360, azimuth_resolution):
    # Convert to radians
    azimuth_rad = math.radians(azimuth)
    
    # Calculate normalized reflection vector (scaled to fixed-point)
    nx = int(math.cos(azimuth_rad) * fixed_point_scale)
    ny = int(math.sin(azimuth_rad) * fixed_point_scale)
    nz = 0  # Assuming 2D LUT for now; adjust for 3D as needed
    
    # Append to LUT
    lut.append((nx, ny, nz))

# Print the LUT in Arduino-compatible format
print("// Precomputed LUT for normalized reflection directions in PROGMEM")
print("// Format: {Nx, Ny, Nz}, where N = normalized vector components * 1000")
print("const int16_t reflectionLUT[360][3] PROGMEM = {")
for i, (nx, ny, nz) in enumerate(lut):
    print(f"    {{{nx}, {ny}, {nz}}},  // Azimuth {i}°")
print("};")
