import numpy as np
import matplotlib.pyplot as plt

# Constants
P0 = 101325        # Sea level standard atmospheric pressure (Pa)
T0 = 288.15        # Sea level standard temperature (K)
L = 0.0065         # Temperature lapse rate (K/m)
g = 9.80665        # Gravity (m/s^2)
M = 0.0289644      # Molar mass of Earth's air (kg/mol)
R_universal = 8.3144598  # Universal gas constant (J/(mol·K))
R_specific = 287.05      # Specific gas constant for dry air (J/(kg·K))

# Altitude range (0 to 11,000 meters)
altitudes = np.linspace(200, 400, 100)

# Specific temperature at altitude (assume constant for simplicity, e.g., 273.15 K = 0°C)
specific_temperature = 273.15 + 20 # Kelvin

# Calculate pressure using barometric formula
pressures = P0 * (1 - (L * altitudes) / T0) ** ((g * M) / (R_universal * L))

# Calculate density using ideal gas law
densities = pressures / (R_specific * specific_temperature)

# Calculate linear gradient (slope) of the density curve
slope = (densities[-1] - densities[0]) / (altitudes[-1] - altitudes[0])
intercept = densities[0] - slope * altitudes[0]
linear_approx = slope * altitudes + intercept

print(f"Gradient (slope) of density curve: {slope:.6e} kg/m³ per meter")

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(altitudes, densities, label=f'Temperature = {specific_temperature} K')
plt.plot(altitudes, linear_approx, '--', label='Linear Approximation (Gradient)')
plt.xlabel('Altitude (m)')
plt.ylabel('Air Density (kg/m³)')
plt.title('Variation of Air Density with Altitude at Specific Temperature')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
