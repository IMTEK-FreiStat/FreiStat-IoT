"""
@file: CV_plotter.py
@author: D. Bill
@brief: Real-time plot of voltammetry data received from the FreiStat (AD5941) potentiostat
        Run this script after flashing the potentiostat MCU (e.g. Adafruit Feather M0) with the voltammetry firmware
        Requires the potentiostat to be connected via USB to the configured serialPort
        Make sure the MCU only sends expected data (no debug info)!!!
"""
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import struct #needed to convert byte object to float
import time
import os
import csv
import numpy as np

# Parameter-Configuration!!!
x_range = [-700, 1400]  # Voltage range in mV --> gets overwritten based on received ramp parameters
y_range = [-50, 50]  # Current range in uA, set it to expected range to see all data in plot during measurement!
serialPort = 'COM3' # Check on which port the MCU is connected
serialBaud = 115200
updatePeriod = 20   #interval in ms at which "animate" fct is called, ensure its smaller than Estep / ScanRate

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.grid()
#array to store received data
xs = [] #voltage of the voltammetry
ys = [] #measured current

# data folder config
work_dir = os.getcwd()
if not os.path.exists('CV_Data'):
    os.mkdir('CV_Data')
work_dir += '/CV_Data/'
os.chdir(work_dir)

# data file config
time_start = time.time()
output_file = time.strftime('%Y%m%d_%H%M%S') + '.csv'

labels = ['Voltage (mV)', 'Current (uA)']

with open(output_file, 'w', newline='') as csvFile:
    writer = csv.writer(csvFile)
    writer.writerow(labels[0:2])
csvFile.close

# Create a blank line, will be updated in animate()
line, = ax.plot(xs, ys)
# Add labels
plt.title('CV')
plt.xlabel('Voltage (mV)')
plt.ylabel('Current (uA)')

# Initialize communication with potentiostat MCU (Adafruit Feather)
serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
serialConnection.port = serialPort
serialConnection.baudrate = serialBaud
serialConnection.timeout = 8    #timeout for read() in seconds

#Open the serial port, this will trigger the potentiostat to run the measurement
print("Attempting to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
while(serialConnection.is_open == False):
    serialConnection.open()
print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
serialConnection.reset_input_buffer()

#Ramp data from Adafruit Feather, will be sent after opening the serial port by running the script
settings = serialConnection.read(size = 18) #settings data is expected to contain 18 bytes
#Unpack settings data into parameters
RampStartVolt = struct.unpack('>h',settings[0:2])[0]
RampPeakVolt1 = struct.unpack('>h',settings[2:4])[0]
RampPeakVolt2 = struct.unpack('>h',settings[4:6])[0]
Estep = struct.unpack('>h',settings[6:8])[0]
ScanRate = struct.unpack('>h',settings[8:10])[0]
CycleNumber = struct.unpack('>h',settings[10:12])[0]
Sinc3OSR = struct.unpack('>h',settings[12:14])[0]
Sinc2OSR = struct.unpack('>h',settings[14:16])[0]
StepNumber = struct.unpack('>h',settings[16:18])[0]

#set new axis range according to ramp parameters
xlim_right = max([RampStartVolt,RampPeakVolt1, RampPeakVolt2]) +50
xlim_left = min([RampStartVolt,RampPeakVolt1, RampPeakVolt2]) -50
ax.set_xlim([xlim_left, xlim_right])
ax.set_ylim(y_range)

step_cnt = 0

# This function is called periodically from FuncAnimation
def animate(i):
    
    global step_cnt #use step_cnt as global variable inside this fct
    
    #Read 2 floats from Adafruit Feather (voltage,current)
    #Wait until required nr. of bytes is read or timeout
    temp = serialConnection.read(size = 8);
    #Unpack data
    voltage = struct.unpack('<f',temp[0:4])[0]
    current = struct.unpack('<f',temp[4:])[0]
    #Print received sample to console
    print(voltage)
    print(current)
    #Add received sample to lists for plotting
    ys.append(current)
    xs.append(voltage)

    # Update line with new values
    line.set_data(xs,ys)
    
    #Attach new sample to .csv file
    with open(output_file, 'a', newline='') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow([voltage,current])
    csvFile.close
    
    step_cnt += 1
    
    #Quit, if expected number of samples is reached
    if(step_cnt == (StepNumber * CycleNumber)):
        #redraw plot with y axis fit to plot
        ax.autoscale(axis = 'y', tight = True)
        
        #different colors for each voltammetry cycle
        x_cycle = np.array_split(xs, CycleNumber)
        y_cycle = np.array_split(ys, CycleNumber)
        
        ax.clear()
        colors = ['r','g','b', 'y', 'c', 'm', 'y', 'k']
        for k in range(CycleNumber):
            ax.plot(x_cycle[k], y_cycle[k], color = colors[k % len(colors)])
        
        ax.grid()
        plt.show()
        #stop the animation
        ani.event_source.stop()
        #close the serial port
        serialConnection.close()


    return line,

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig,
    animate,
    interval=updatePeriod,
    blit=True)
plt.show()
