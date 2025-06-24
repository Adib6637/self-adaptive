import pandas as pd
import matplotlib.pyplot as plt

file_path = '../Simulation/dataset/flights.csv'  # Change this to the path of your CSV file
import os
df = pd.read_csv(file_path)

df.columns = [
    "flight", "time", "wind_speed", "wind_angle", "battery_voltage", "battery_current",
    "position_x", "position_y", "position_z",
    "orientation_x", "orientation_y", "orientation_z", "orientation_w",
    "velocity_x", "velocity_y", "velocity_z",
    "angular_x", "angular_y", "angular_z",
    "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z",
    "speed", "payload", "altitude", "date", "time_day", "route"
]

name = 'battery_current'
print(name)
print("min: " + str(df[name][:].min()))
print("max: " + str(df[name][:].max()))

max_index = df[name].idxmax()
print("Index of max value:", max_index)
print("Row with max value:\n", df.loc[max_index])

#plot
plt.figure(figsize=(10, 6))
plt.plot(df[name][:], label=name, color='blue')
plt.xlabel(name, fontsize=12)
plt.show()


#print("min: " + str(((df['battery_voltage']*df['battery_current']).min())))
#print("max: " + str(((df['battery_voltage']*df['battery_current']).max())))
#print("min: " + str(((df['velocity_x']*df['velocity_y']*df['velocity_z'])**(1/3)).min()))
#print("max: " + str(((df['velocity_x']*df['velocity_y']*df['velocity_z'])**(1/3)).max()))
#array = df[name].values
#tmp = []
#for v in array:
#    if v < 4:
#        tmp.append(float(v))
#print("min: " + str(min(tmp)))
#print("max: " + str(max(tmp)))
#tmp = ((df['battery_voltage']*df['battery_current']))
#plt.ylim(-100, 10)