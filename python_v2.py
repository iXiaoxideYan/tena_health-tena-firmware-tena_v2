import serial
import matplotlib
matplotlib.use('GTK3Agg')
import matplotlib.pyplot as plt

def plot_serial_data():
    # Initialize UART connection
    ser = serial.Serial('COM14', 115200, timeout=5)

    # Initialize figure for both subplots
    fig, (ax1, ax2) = plt.subplots(2, 1)
    # Set chart formatting for accelerometer data
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Accelerometer Value')
    # Set chart formatting for gyroscope data
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Gyroscope Value')
    # Initialize lists to store data
    time_data = []
    acc_data = []
    gyro_data = []

    print("Start Recording")

    #Init the graph before drawing
    fig.canvas.draw()
    # Infinite loop to read data from UART and plot data on charts
    while True:
        # Read data from UART
        line = ser.readline().decode('latin-1').rstrip()
        # Parse data
        try:
            # Limit the data to the last 5000 samples
            if len(time_data) > 5000:
                time_data = time_data[-5000:]
                acc_data = acc_data[-5000:]
                gyro_data = gyro_data[-5000:]

            # Remove "data:" header and split line into values
            values = line.split(", ")
            values.pop(0)
            # Update charts
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = values
            time_data.append(len(acc_data))
            acc_data.append((float(acc_x), float(acc_y), float(acc_z)))
            gyro_data.append((float(gyro_x), float(gyro_y), float(gyro_z)))
            # Plot accelerometer data
            ax1.clear()
            ax2.clear()
            ax1.set_ylim([-2, 2])
            ax1.plot(time_data, [data[0] for data in acc_data], label='Accelerometer X', color='red')
            ax1.plot(time_data, [data[1] for data in acc_data], label='Accelerometer Y', color='green')
            ax1.plot(time_data, [data[2] for data in acc_data], label='Accelerometer Z', color='blue')
            # Plot gyroscope data
            ax2.set_ylim([-250, 250])
            ax2.plot(time_data, [data[0] for data in gyro_data], label='Gyroscope X', color='red')
            ax2.plot(time_data, [data[1] for data in gyro_data], label='Gyroscope Y', color='green')
            ax2.plot(time_data, [data[2] for data in gyro_data], label='Gyroscope Z', color='blue')
            # Draw and blit the updated plot
            ax1.legend()
            ax2.legend()
            fig.canvas.draw_idle()
            fig.canvas.blit(ax1.bbox)
            fig.canvas.blit(ax2.bbox)
            fig.canvas.flush_events()
            plt.pause(0.00001)
        except ValueError:
            pass

while True:
    plot_serial_data()