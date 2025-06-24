import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
file_path = '../log/log_loss.csv'  # Change this to the path of your CSV file
#file_path = '../Simulation/log/log_loss.csv'  # Change this to the path of your CSV file
df = pd.read_csv(file_path)

# Assuming the columns are named 'Model_1' and 'Model_2' (adjust names as necessary)
plt.figure(figsize=(5, 5))

df.columns = ['Model_1', 'Model_2','counter']
#df_subset = df.iloc[:500]
df_subset = df[100:500].reset_index(drop=True)
print(df_subset['counter'].min(), df_subset['counter'].max())

# Plotting the loss values
#plt.plot(df_subset['Model_1'], label='Model 1 Loss', color='blue')
plt.plot(df_subset['Model_2'], label='Model 2 Loss', color='red')
#plt.plot(df_subset['counter'], label='counter', color='green')

# Adding titles and labels
#plt.title('Loss value of actuator power model ')#'Loss Value Comparison of Two Models', fontsize=14)
plt.title('Loss value of sensor power model ')#'Loss Value Comparison of Two Models', fontsize=14)
plt.xlabel('Epochs', fontsize=12)
plt.ylabel('Loss Value', fontsize=12)
#plt.legend()


#plt.ylim(-100, 10)
# Show the plot
plt.show()
