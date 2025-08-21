import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
import time

# Enable interactive mode
plt.ion()

# Change the working directory to the script's directory
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# Create figure and axis objects
fig, ax = plt.subplots(figsize=(6, 6))
line, = ax.plot([], [], 'r-', label='Actuator Model Loss')
ax.set_xlabel('Iteration', fontsize=12)
ax.set_ylabel('Loss Value', fontsize=12)
ax.set_title('Loss value of Actuator power model')
ax.legend()

# Initialize empty data
data = []
last_size = 0

try:
    while True:
        # Read the CSV file
        file_path = '../log/log_loss.csv'
        if os.path.exists(file_path):
            df = pd.read_csv(file_path)[10:200]
            df.columns = ['actuator', 'sensor', 'counter']
            
            # Only update if there's new data
            if len(df) > last_size:
                # Clear the axis
                ax.clear()
                
                # Plot new data
                ax.plot(df['actuator'], 'r-', label='Actuator Model Loss')
                
                # Update labels and title
                ax.set_xlabel('Iteration', fontsize=12)
                ax.set_ylabel('Loss Value', fontsize=12)
                ax.set_title('Loss value of Actuator power model')
                ax.legend()
                
                # Update display
                fig.canvas.draw()
                fig.canvas.flush_events()
                
                # Update last size
                last_size = len(df)
                
                # Print min and max counter values
                print(f"Counter range: {df['counter'].min()} to {df['counter'].max()}")
        
        # Wait before next update
        time.sleep(1)  # Update every second

except KeyboardInterrupt:
    print("\nStopping visualization...")
    plt.ioff()
    plt.close('all')


"""
# Change the working directory to the script's directory
script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# Read the CSV file
file_path = '../log/log_loss.csv' 
df = pd.read_csv(file_path)

plt.figure(figsize=(6, 6))

columns = ['actuator', 'sensor','counter']
df.columns = columns
df_subset = df[:].reset_index(drop=True)
print(df_subset['counter'].min(), df_subset['counter'].max())

# Plotting the loss values
select_model = columns[0]
plt.plot(df_subset[select_model], label= select_model+' Model Loss', color='red')

# Adding titles and labels
plt.title('Loss value of ' + select_model + ' power model ')
plt.xlabel('Iteration', fontsize=12)
plt.ylabel('Loss Value', fontsize=12)
plt.show()
"""