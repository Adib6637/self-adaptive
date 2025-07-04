import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
file_path = '../log/log_loss.csv' 
df = pd.read_csv(file_path)

plt.figure(figsize=(6, 6))

df.columns = ['actuator', 'sensor','counter']
df_subset = df[160:200].reset_index(drop=True)
print(df_subset['counter'].min(), df_subset['counter'].max())

# Plotting the loss values
select_model = 'actuator'
plt.plot(df_subset[select_model], label= select_model+' Model Loss', color='red')

# Adding titles and labels
plt.title('Loss value of ' + select_model + ' power model ')
plt.xlabel('Iteration', fontsize=12)
plt.ylabel('Loss Value', fontsize=12)
plt.show()
