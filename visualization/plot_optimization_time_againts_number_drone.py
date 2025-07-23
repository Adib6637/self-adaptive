import matplotlib.pyplot as plt

# Data
x = [3, 5, 10]
y = [0.174949, 3.079105, 51.542050]
y_min = [0.134199, 0.951563, 27.400800]
y_max = [0.220292, 9.601460, 77.554400]

# Calculate asymmetric error bars
y_err_lower = [y_i - y_min_i for y_i, y_min_i in zip(y, y_min)]
y_err_upper = [y_max_i - y_i for y_i, y_max_i in zip(y, y_max)]

# Plot
plt.errorbar(x, y, yerr=[y_err_lower, y_err_upper], fmt='o', capsize=5, label='Data with Error Bars')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plot with Asymmetric Error Bars')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
