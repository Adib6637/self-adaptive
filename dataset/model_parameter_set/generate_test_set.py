import pandas as pd
import matplotlib.pyplot as plt


file_path_actuator_coefficients = '../Simulation/log/log_coefficient_actuator.csv'
file_path_sensor_coefficients = '../Simulation/log/log_coefficient_sensor.csv'

df_actuator = pd.read_csv(file_path_actuator_coefficients)
df_sensor = pd.read_csv(file_path_sensor_coefficients)

df_actuator_subset = df_actuator.iloc[:400,0:4].reset_index(drop=True)
df_sensor_subset = df_sensor.iloc[100:500,0:6].reset_index(drop=True)

column_names = [
    'actuator_col1', 'actuator_col2', 'actuator_col3', 'actuator_col4', 
    'sensor_col1', 'sensor_col2', 'sensor_col3', 'sensor_col4', 'sensor_col5', 'sensor_col6'
]
df_new = pd.concat([df_actuator_subset, df_sensor_subset], axis=1)
df_new.columns = column_names
# Save to a new CSV file
df_new.to_csv('../Simulation/dataset/model_parameter_set/model_coefficient_test_set.csv', index=False)


