import pandas as pd
import matplotlib.pyplot as plt

# Load data from CSV file into a pandas dataframe
df = pd.read_csv('sensor_data.csv')

# Get data from dataframe columns
acc_x = df['acc_x']
acc_y = df['acc_y']
acc_z = df['acc_z']
gyro_x = df['gyro_x']
gyro_y = df['gyro_y']
gyro_z = df['gyro_z']

# Plot the data using matplotlib
fig, (ax1, ax2) = plt.subplots(nrows=2, sharex=True, figsize=(12, 8))
ax1.plot(acc_x, label='Acc X')
ax1.plot(acc_y, label='Acc Y')
ax1.plot(acc_z, label='Acc Z')
ax1.set_ylabel('Accelerometer Value')
ax1.legend()
ax2.plot(gyro_x, label='Gyro X')
ax2.plot(gyro_y, label='Gyro Y')
ax2.plot(gyro_z, label='Gyro Z')
ax2.set_ylabel('Gyroscope Value')
ax2.legend()
plt.show()