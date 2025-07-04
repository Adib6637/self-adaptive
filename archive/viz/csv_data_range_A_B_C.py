import pandas as pd
import matplotlib.pyplot as plt

file_path = '../../../OneDrive/FH Dortmund/Sem4/Master-Thesis/Implementation/Simulation/log/log_A_B_C.csv'  # Change this to the path of your CSV file
df = pd.read_csv(file_path)


df.columns = ['A','B','C','data_number'];


name = 'B'
print(name)
print("min: " + str(df[name][:].min()))
print("max: " + str(df[name][:].max()))


#plot
plt.figure(figsize=(10, 6))
plt.plot(df[name][:], label=name, color='blue')
plt.xlabel(name, fontsize=12)
#plt.ylim(-100, 10)
plt.show()
