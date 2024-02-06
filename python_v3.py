import csv
import datetime
import serial
import matplotlib
import re
from matplotlib.animation import FuncAnimation
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# Generate a unique file name based on the current date and time
timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
filename = f'sensor_data_{timestamp}.csv'

# create a CSV file
csv_file = open('sensor_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'])


# Initialize list to store last 300 samples
max_samples = 300

# Initialize the plot
fig, (ax1, ax2) = plt.subplots(nrows=2, sharex=True, figsize=(12, 8))

# Initialize lists to store data
sample_count = []
acc_x_data = []
acc_y_data = []
acc_z_data = []
gyro_x_data = []
gyro_y_data = []
gyro_z_data = []

# Set chart formatting for accelerometer data
ax1.set_xlabel('Samples')
ax1.set_ylabel('Accelerometer Value')
ax1.set_ylim([-2, 2])

# Set chart formatting for gyroscope data
ax2.set_xlabel('Samples')
ax2.set_ylabel('Gyroscope Value')
ax2.set_ylim([-250, 250])

# Initialize UART connection
ser = serial.Serial('COM14', 115200, timeout=5)

print("Start Recording")

# Function to read and parse data from UART
def read_data():
    ser.timeout = 0.1
    lines = ser.readlines()
    for line in lines:
        line = line.decode('latin-1').rstrip()
        try:
            if line:
                print(line)
                pattern = r"[-+]?\d*\.\d+|\d+"
                values = re.findall(pattern, line)
                if any([float(v) > 500 for v in values]):
                    continue
                # Update data lists
                acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = values
                acc_x_data.append(float(acc_x))
                acc_y_data.append(float(acc_y))
                acc_z_data.append(float(acc_z))
                gyro_x_data.append(float(gyro_x))
                gyro_y_data.append(float(gyro_y))
                gyro_z_data.append(float(gyro_z))
                # Write data to CSV file
                csv_writer.writerow([acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])
                # Update sample count
                sample_count.append(len(sample_count) + 1)
        except ValueError:
            print("Error")
            print(line)

# Function to update plot with new data
def update_plot(frame):
    # Read and parse data from UART
    read_data()
    # Update plot data
    ax1.clear()
    ax2.clear()
    ax1.set_ylim([-2, 2])
    ax2.set_ylim([-250, 250])
    ax1.plot(sample_count[-max_samples:], acc_x_data[-max_samples:], label='Acc X')
    ax1.plot(sample_count[-max_samples:], acc_y_data[-max_samples:], label='Acc Y')
    ax1.plot(sample_count[-max_samples:], acc_z_data[-max_samples:], label='Acc Z')
    ax2.plot(sample_count[-max_samples:], gyro_x_data[-max_samples:], label='Gyro X')
    ax2.plot(sample_count[-max_samples:], gyro_y_data[-max_samples:], label='Gyro Y')
    ax2.plot(sample_count[-max_samples:], gyro_z_data[-max_samples:], label='Gyro Z')
    ax1.legend()
    ax2.legend()

# Animate plot
ani = FuncAnimation(fig, update_plot, interval=50)
plt.show()
