import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
file_path = '../log/log_coefficient_sensor.csv'
df = pd.read_csv(file_path)

plt.figure(figsize=(10, 6))

df.columns = ['sigma', 'omega', 'epsilon', '', '', '','counter']
df_subset = df.iloc[:]

# Plotting the loss values
plt.plot(df_subset['sigma'], label='eta', color='blue')
plt.plot(df_subset['omega'], label='delta', color='red')
plt.plot(df_subset['epsilon'], label='alpha', color='green')

# Adding titles and labels
plt.title('')#'Loss Value Comparison of Two Models', fontsize=14)
plt.xlabel('Epochs', fontsize=12)
plt.ylabel('Coefficient Value', fontsize=12)
plt.legend()

# Show the plot
plt.show()
