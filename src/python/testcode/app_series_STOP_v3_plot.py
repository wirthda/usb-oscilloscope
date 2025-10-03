# file: app_series_STOP_v3_plot.py
# 
# This program traverses through the FSM of the µC INCLUDING DUMMY DATA RECEPTION.
# The FSM is controlled with string-commands send over the emulated Serial Interface (hardware: USB FS).
# 
# used driver on µC: USB CDC Device (STM32F7);
# 
# works with STM32CubeIDE-project "USB_Oszi_Firmware_V1"
#
# Beginning: FSM is in Initial State (Init)
# "APP\n" starts the application (Init->Idle)
# after execution of the program, the FSM is in Idle State
# 
# single commands can be easily sent via hterm-software
# https://www.der-hammer.info/pages/terminal.html
# Sent commands need to include a EOL-char (\n or \r) --> setting "send on enter XX" XX = LF or CR

import time
import serial
import matplotlib.pyplot as plt

def shift(a):
    return a >> 3

def mask(a):
    return a & 0xFFF

def twosComplToSignedInt(a):
    if(((a >> 11) & 1) == 1):
        a -= 1
        a ^= 0xFFF
        a = -a
    return a

datalength = 200000

# x-axis: indices of the list
indices = list(range(datalength//2))

#--configure port--
ser = serial.Serial()

# baud rate is emulated!
# real transmission speed is determined by USB speed (USB FS: 12MBit/s)
ser.baudrate = 115200

# timeout is needed for read()-/readline()-functions (e.g ACQUISITION:DONE)
ser.timeout = 5

# COM-port on Windows PC (VCP)
ser.port = 'COM13'

#print information (see default values of parameters, that were not explicitly set)
print(ser)

# open serial port
ser.open()
print('--Port opened: {0}--'.format(ser.is_open))
print('')

# reset
print('--RESET--')
print("out: \"RESET\"")
ser.write(b'RESET\n')
print('')

# start application of µC
print('--Start app--')
print("out: \"APP\"")
ser.write(b'APP\n')
# get acknoledgement (STATE:IDLE)
line = ser.readline()
print("in: {0}".format(line))
print('')

# configure trigger
print('--Configure trigger--')
print("out: \"CONFIGURE_TRIG_FALL_EDGE\"")
ser.write(b'CONFIGURE_TRIG_FALL_EDGE\n')
# get acknoledgement (TRIGGER:XXX_EDGE)     XXX=RISING, FALLING
line = ser.readline()
print("in: {0}".format(line))
print('')

print('--Start acquisiton (waits for trigger)--')
print("out: \"RUN_START\"")
ser.write(b'RUN_START\n')
# get acknoledgement (STATE:ACQUISITION_START)
line = ser.readline()
print("in: {0}".format(line))

print('--wait for end of acquisition--')
# get acknoledgement (STATE:ACQUISITION_DONE)
line = ser.readline()
print("in: {0}".format(line))

print('')

print('--Signal readiness for data reception--')
print("out: \"READY_FOR_DATA\"")
ser.write(b'READY_FOR_DATA\n')
# get acknoledgement (STATE:TRANSMISSION) !currently not sent by firmware!
# line = ser.readline()
# print("in: {0}".format(line))

# Data reception (100000 values -> 200000 bytes)
time1 = time.perf_counter()
time11 = time.time()
data = ser.read(datalength)
time2 = time.perf_counter()
time21 = time.time()
print('Transmission time: {0}'.format(time2 - time1))
print('Transmission time (alt): {0}'.format(time21 - time11))
print('Received Bytes: {0}'.format(len(data)))

# Convert byte string to list of 2byte-integers (data has same order as in µC memory)
time1 = time.perf_counter()
data_list = [
    int.from_bytes(data[i:i+2], byteorder='little')  # or 'big'
    for i in range(0, len(data), 2)
]
#for i in range(2000):
#    binary = format(data_list[i], 'b')
#    print(binary, end=" ")
data_list_shifted = list(map(shift, data_list))
data_list_masked = list(map(mask, data_list_shifted))
data_list_processed = list(map(twosComplToSignedInt, data_list_masked))
time2 = time.perf_counter()
print('Conversion time: {0}'.format(time2 - time1))
#print("{0}".format(data_list_processed))

print('')

print('--Acknowledge reception--')
print("out: \"RECEIVED\"")
ser.write(b'RECEIVED\n')
# get acknoledgement (STATE:WAIT_FOR_ACK)
line = ser.readline()
print("in: {0}".format(line))

print('')

print('--send ACK to µC: STOP--')
print("out: \"STOP\"")
ser.write(b'STOP\n') # alternative: CONTINUE -> back in STATE:ACQUISITION
# get acknoledgement (STATE:IDLE)
line = ser.readline()
print("in: {0}".format(line))

# close serial port
ser.close()

# Create the plot
plt.figure(figsize=(8, 4))
plt.plot(indices, data_list_processed, linestyle='-', color='blue')

# Add labels and title
plt.xlabel('index')
plt.ylabel('data value')
plt.title('list of data (index on X-axis)')
plt.grid(True)

# Show the plot
plt.show()