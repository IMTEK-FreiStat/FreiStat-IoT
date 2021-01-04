"""
@file: CA_plotter.py
@author: D. Bill
@brief: Real-time plot of chronoamperometry data received from the FreiStat (AD5941) potentiostat
        Run this script after flashing the potentiostat MCU (e.g. Adafruit Feather M0) with the chronoamperometry firmware
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
y_range = [-60, 60]   # Current range in uA, set it to expected range to see all data in plot during measurement!
serialPort = 'COM3'
serialBaud = 115200
updatePeriod = 1   #interval at which "animate" fct is called, should be smaller than the data output rate of the potentiostat
MAX_STEPS = 200  # set it to MAX_STEPS makro in ChronoAmperometric.h - number of entries in the pulseAmplitude array that will be sent along config data
FifoThresh = 10   #set it to FifoThresh parameter in the AD5940AMPStructInit() function - determines how many samples are averaged and thus sets the time scale of incoming samples

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.grid()
#array to store received data
xs = [] #time
ys = [] #measured current
PulseVoltage = []   #list containing the exact pulse voltages in mV, will be filled with values sent from MCU

# data folder config
work_dir = os.getcwd()
if not os.path.exists('CA_Data'):
    os.mkdir('CA_Data')
work_dir += '/CA_Data/'
os.chdir(work_dir)

# data file config
time_start = time.time()
output_file = time.strftime('%Y%m%d_%H%M%S') + '.csv'

labels = ['Current (uA)']

with open(output_file, 'w', newline='') as csvFile:
    writer = csv.writer(csvFile)
    writer.writerow(labels[0:1])
csvFile.close

# Create a blank line. We will update the line in animate
line, = ax.plot(xs, ys)
# Add labels
plt.title('CA')
plt.xlabel('Time (s)')
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

#CA config data from Adafruit Feather, will be sent after opening the serial port by running the script
settings = serialConnection.read(size = (MAX_STEPS*4 + 8))
#Unpack settings data into parameters
i = 0
while (i < (MAX_STEPS*4)):
    volt = struct.unpack('<f',settings[i:(i+4)])[0]
    PulseVoltage.append(volt)
    i = i + 4
StepNumber = struct.unpack('>h',settings[(MAX_STEPS*4):(MAX_STEPS*4 + 2)])[0]
Sinc3OSR = struct.unpack('>h',settings[(MAX_STEPS*4 + 2):(MAX_STEPS*4 + 4)])[0]
Sinc2OSR = struct.unpack('>h',settings[(MAX_STEPS*4 + 4):(MAX_STEPS*4 + 6)])[0]
CA_duration = struct.unpack('>H',settings[(MAX_STEPS*4 + 6):(MAX_STEPS*4 + 8)])[0]

#time between 2 samples in s
delta_t = 1/(800000/Sinc3OSR/Sinc2OSR)
#certain number of samples is averaged before sending
delta_t = delta_t * FifoThresh

#set new axis range according to ramp parameters
ax.set_xlim([0, CA_duration])
ax.set_ylim(y_range)

current_time = 0
sample_cnt = 0

# This function is called periodically from FuncAnimation
def animate(i):
    
    global current_time #use current_time as global variable inside this fct
    global sample_cnt #use sample_cnt as global variable inside this fct
    
    #read one float from Adafruit Feather (average current)
    temp = serialConnection.read(size = 4);
    #check if no bytes where received (means that CA is finished)
    if (len(list(temp))  == 0):
        #redraw plot with y axis fit to plot
        ax.autoscale(axis = 'y', tight = True)
        ax.clear()
        ax.plot(xs, ys)
        ax.grid()
        plt.show()
         #stop the animation
        ani.event_source.stop()
        #close the serial port
        serialConnection.close()
    else:    
        #print(temp)
        current = struct.unpack('<f',temp[0:])[0]
        print(current)
        #Add received sample to lists for plotting
        ys.append(current)
        xs.append(current_time)
    
        # Update line with new values
        line.set_data(xs,ys)
        
        #Attach new sample to .csv file
        with open(output_file, 'a', newline='') as csvFile:
            writer = csv.writer(csvFile)
            writer.writerow([current_time,current])
        csvFile.close
            
        current_time = current_time + delta_t
        sample_cnt = sample_cnt + 1

    return line,

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig,
    animate,
    interval=updatePeriod,
    blit=True)
plt.show()
