import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Serial port configuration
SERIAL_PORT = 'COM3'  # Change to your serial port
BAUD_RATE = 1000000

# Frequency domain configuration
SAMPLE_RATE = 44100  # Sampling frequency in Hz
FFT_SIZE = 1024  # Size of FFT (number of samples)

# Initialize serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("Serial port opened successfully.")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

# Initialize plot
fig, ax = plt.subplots()
xdata, ydata = [], []
line, = ax.plot([], [], lw=2)

def init():
    ax.set_xlim(0, SAMPLE_RATE // 2)
    ax.set_ylim(0, 500000)  # Adjust as needed based on expected amplitude
    return line,

def update(frame):
    global xdata, ydata
    try:
        # Read data from serial port
        data = ser.read(FFT_SIZE * 2)  # Read 1024 samples (16 bits each)
        if len(data) == FFT_SIZE * 2:
            # Convert data to numpy array
            samples = np.frombuffer(data, dtype=np.int16)
            # Perform FFT
            fft_result = np.fft.fft(samples)
            fft_freqs = np.fft.fftfreq(len(fft_result), 1 / SAMPLE_RATE)
            # Get magnitude spectrum
            magnitude = np.abs(fft_result)
            # Update plot data
            xdata = fft_freqs[:FFT_SIZE // 2]
            ydata = magnitude[:FFT_SIZE // 2]
            line.set_data(xdata, ydata)
            ax.relim()
            ax.autoscale_view()
    except Exception as e:
        print(f"Error during update: {e}")
    return line,

# Ensure the plot window stays open
try:
    ani = FuncAnimation(fig, update, init_func=init, blit=True, interval=50, cache_frame_data=False)
    plt.show()
except Exception as e:
    print(f"Error with animation: {e}")

# Ensure the script doesn't exit immediately
while True:
    try:
        plt.pause(0.1)
    except Exception as e:
        print(f"Error in the main loop: {e}")
        break

# Close the serial port when the script exits
ser.close()
