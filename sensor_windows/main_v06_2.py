# Title : uSkin Server
# Date  : 2017/11/23
# Author: Shu


import sys, os, time, types, csv, string, thread
import getopt, util, thread, threading, numpy
import pickle, socket, canopen, ntcan, ipdb, struct
from copy import deepcopy
from shutil import copyfile
from multiprocessing.connection import Listener, Client



###########################
## Globle Configurations ##
###########################

# Board Settings
board_start_num = 1
num_of_board    = 7
num_of_tip      = 0
num_of_axis     = 3
num_of_taxel    = 16

# Buffer and array initialisation
id_base     = 0x200     #Target ID of the MTB that we want to trigger
cif_array   = {}        # NTcan cif array
cmsg_array  = {}        # NTcan cmsg array
mlx_buffer  = {}        # MLX reading buffer accomodate up to 16 taxels

# Buffer initialisation for TCP/IP communication
data_length = 6  # (X Y Z) * (MSB & LSB)
size        = num_of_taxel * data_length * num_of_board
step        = num_of_taxel * data_length
byte_array  = bytearray(size)
byte_buffer = buffer(byte_array, 0, size)
shift_array = [None] * (num_of_board) * num_of_taxel * num_of_axis

# Limitation initialisation and settings
tracking        = numpy.zeros((num_of_board, num_of_taxel, num_of_axis))
difference      = numpy.zeros((num_of_axis))
limit_threshold = 0xA0
limit_upper     = 0xFA
limit_lower     = 0x0A

# Setup TCP server
ogasa = '192.168.11.13'
proxy = '192.168.11.36'
local = '127.0.0.1'
TCP_IP = proxy
TCP_PORT = 5008

# CAN connection configurations
net  = 2        # Logical CAN Network [0, 255]
RxQS = 1        # RxQueueSize [0, 10000]
RxTO = 2000     # RxTimeOut in Millisconds
TxQS = 1        # TxQueueSize [0, 10000]
TxTO = 1000     # TxTimeOut in Millseconds


##########################
## Function Definitions ##
##########################

# Remove previous CSV log files
def csv_log_init():

    # Remove old baseline log files
    for i in range(num_of_board):
        del_num = board_start_num + i

        if os.path.exists('LOG%s.csv' % del_num):
            os.remove('LOG%s.csv' % del_num)


# Create sensor debug logs and its header row
def csv_debug_processed_init():

    # Initialising taxel number header
    tactile_header = []
    for i in range(num_of_board - num_of_tip):
        for j in range(num_of_taxel):
            tactile_header.append(str(i)+'B'+str(j)+'X')
            tactile_header.append(str(i)+'B'+str(j)+'Y')
            tactile_header.append(str(i)+'B'+str(j)+'Z')

    if num_of_tip == 1:
        for i in range(num_of_board - num_of_tip, num_of_board):
            for j in range(num_of_taxel / 2):
                tactile_header.append(str(i)+'B'+str(j)+'X')
                tactile_header.append(str(i)+'B'+str(j)+'Y')
                tactile_header.append(str(i)+'B'+str(j)+'Z')

    # Initialising CSV header
    with open('visualization/debug_processed.csv', 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(tactile_header)


# Write to debug log file
def csv_debug_processed_write(array):

    # Write to CSV for debugging
    with open('visualization/debug_processed.csv', 'a') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(array)


# Create sensor debug logs and its header row
def csv_debug_raw_init():

    # Initialising taxel number header
    tactile_header = []
    for i in range(num_of_board - num_of_tip):
        for j in range(num_of_taxel):
            tactile_header.append(str(i)+'B'+str(j)+'X')
            tactile_header.append(str(i)+'B'+str(j)+'Y')
            tactile_header.append(str(i)+'B'+str(j)+'Z')

    if num_of_tip == 1:
        for i in range(num_of_board - num_of_tip, num_of_board):
            for j in range(num_of_taxel / 2):
                tactile_header.append(str(i)+'B'+str(j)+'X')
                tactile_header.append(str(i)+'B'+str(j)+'Y')
                tactile_header.append(str(i)+'B'+str(j)+'Z')

    # Initialising CSV header
    with open('visualization/debug_raw.csv', 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(tactile_header)


def csv_debug_raw_write(array):

    # Write to CSV for debugging
    with open('visualization/debug_raw.csv', 'a') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(array)


# NTcan buffer array initialisation
def ntcan_init():

    # cif = ntcan.CIF( net, RxQueueSize, RxTimeOut, TxQueueSize, TxTimeOut, Flags)
    # RxQS, TxQS = 1 for real time application

    # Initialise cif using a loop
    for j in range(board_start_num, board_start_num + num_of_board):
        for k in range(0, num_of_taxel):
            cif_array[j, k] = ntcan.CIF(net, RxQS)

            # CAN-API-Description & set baudrate 0 = 1MBaud
            cif_array[j,k].baudrate = 0

            # Create CAN-Message structure
            cmsg_array[j,k] = ntcan.CMSG()

        # Target ID of the MTB & trigger the MTB to return the data
        # canWrite...(cif, can-id, len, data ...)
        cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)

    print('\nSetting up ESD-CAN...')
    print(cif_array[board_start_num, 0])
    print(('Cmsg: %s') % (cmsg_array[board_start_num, 0]))
    print(('Cmsg lost: %d') % (cmsg_array[board_start_num, 0].msg_lost))


# Generate CAN address
def can_init():

    can_temp = []
    can_addr = []
    header_ID = 0x7

    for j in range(board_start_num, board_start_num + num_of_board):
        for k in range(0, num_of_taxel):
            can_temp.append(header_ID << 8 | j << 4 | k)

        can_addr.append(can_temp)
        can_temp = []

        for k in range(0, num_of_taxel):
            cif_array[j, k].canIdAdd(can_addr[j - board_start_num][k])

    print('Setup is completed\n')


# Read MXL with general configurations
def read_sensor(start_no, end_no, taxel_no):

    try:
        for j in range (start_no, end_no):
            cmsg_array[j, 0].canWriteByte(cif_array[j, 0],(id_base | j), 2, 7, 0)
            for k in range (0, taxel_no):
                cmsg_array[j,k].canRead(cif_array[j, k])

    except IOError, (errno):
        print "I/O Error (%s): %s " % (errno, j)


# Start burst 16 taxel board (only work with 1 board)
def start_sensor_all():

    try:
        for j in range (board_start_num, board_start_num + num_of_board - num_of_tip):
            cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)

        if num_of_tip == 1:
            for j in range (5, 6):
                cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)

    except IOError, (errno):
        print "I/O Error (%s): %s " % (errno, j)


# Read 16 taxel board (with canWriteByte activated reading is limited to 60 hz)
def read_sensor_all():

    try:
        for j in range (board_start_num, board_start_num + num_of_board - num_of_tip):
            cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)
            for k in range (0, num_of_taxel):
                cmsg_array[j,k].canRead(cif_array[j, k])

        time.sleep(0.008)
        if num_of_tip == 1:
            for j in range (5, 6):
                cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)
                for k in range (0, 8):
                    cmsg_array[j,k].canRead(cif_array[j, k])

    except IOError, (errno):
        print "I/O Error (%s): %s " % (errno, j)


# Transfer all stored CAN message reading into MLX buffer
def mlx_buffer_cache():

    for j in range (0, num_of_board):
        for k in range (0, num_of_taxel):
            mlx_buffer[j, k] = cmsg_array[board_start_num + j, k].data.c

    return mlx_buffer


# Combine MSB & LSB into log buffer with general setting
def bit_shift(buffer_in, start_no, end_no, taxel_no):

    # Accomodate up to 16 taxels
    x_axis  = {}
    y_axis  = {}
    z_axis  = {}
    bit_buffer = {}

    for j in range (start_no, end_no):
        bit_buffer[j] = list()
        for k in range (0, taxel_no):
            x_axis[j,k] = buffer_in[j,k][1] << 8 | buffer_in[j,k][2]
            y_axis[j,k] = buffer_in[j,k][3] << 8 | buffer_in[j,k][4]
            z_axis[j,k] = buffer_in[j,k][5] << 8 | buffer_in[j,k][6]
            bit_buffer[j].extend([x_axis[j, k], y_axis[j, k], z_axis[j, k]])

    return bit_buffer


# Combine MSB & LSB into log buffer for 16 taxel board
def bit_shift_all(buffer_in):

    # Accomodate up to 16 taxels
    x_axis  = {}
    y_axis  = {}
    z_axis  = {}
    bit_buffer = {}

    for j in range (0, num_of_board - num_of_tip):
        bit_buffer[j] = list()
        for k in range (0, num_of_taxel):
            x_axis[j,k] = buffer_in[j,k][1] << 8 | buffer_in[j,k][2]
            y_axis[j,k] = buffer_in[j,k][3] << 8 | buffer_in[j,k][4]
            z_axis[j,k] = buffer_in[j,k][5] << 8 | buffer_in[j,k][6]
            bit_buffer[j].extend([x_axis[j, k], y_axis[j, k], z_axis[j, k]])

    if num_of_tip == 1:
        for j in range (4, 5):
            bit_buffer[j] = list()
            for k in range (0, 8):
                x_axis[j,k] = buffer_in[j,k][1] << 8 | buffer_in[j,k][2]
                y_axis[j,k] = buffer_in[j,k][3] << 8 | buffer_in[j,k][4]
                z_axis[j,k] = buffer_in[j,k][5] << 8 | buffer_in[j,k][6]
                bit_buffer[j].extend([x_axis[j, k], y_axis[j, k], z_axis[j, k]])

    return bit_buffer


# Record baseline
def record_baseline():

    print('Triggering sensor...')

    # Trigger first 100 readings to avoid 0xFF initalization reading
    for i in range (0, 100):
        read_sensor_all()
        time.sleep(0.01)

    print('Recording baseline...')
    # Record for boards of 16 taxel
    for j in range(0, num_of_board - 0):
        write_num = board_start_num + j
        csvfile = open('visualization/LOG%s.csv' % write_num, 'wb')
        filewrite = csv.writer(csvfile)

        for i in range (0, 100):
            read_sensor_all()
            time.sleep(0.01)
            mlx_buffer_cache()
            bit_buffer = bit_shift_all(mlx_buffer)
            filewrite.writerow(bit_buffer[j])

            sys.stdout.write('\rBorad No: %s | Recording progress: %s %%' % (j + 1, i + 1))
            sys.stdout.flush()

        csvfile.close()

    #copyfile('visualization/LOG3.csv', 'visualization/LOG1.csv')
    #copyfile('visualization/LOG1.csv', 'visualization/LOG2.csv')
    print('\nFin#ished')


def server_init():

    #raw_input('\nPress Enter to start TCP/IP server...')

    # Start TCP server & wait for connection
    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s.bind((TCP_IP, TCP_PORT))
    #s.listen(1)

    server = Listener((TCP_IP, TCP_PORT))
    print('Server started, waiting for client conection...')

    #conn, addr = s.accept()
    conn = server.accept()
    print('Client accepted!')
    print('Press ctrl + c to stop')

    return conn


# Limitation process to prevent overflow of 16 bit data
def limitation_handler(previous_buffer, current_buffer, publish_buffer):

    for j in range(0, num_of_board):
        for k in range(0, num_of_taxel):
            difference[0] = previous_buffer[j, k][1] - current_buffer[j, k][1]
            difference[1] = previous_buffer[j, k][3] - current_buffer[j, k][3]
            difference[2] = previous_buffer[j, k][5] - current_buffer[j, k][5]

            # Handle tracker depending on difference
            for i in range(0, num_of_axis):
                if difference[i] >= limit_threshold:
                    tracking[j][k][i] += 1
                elif difference[i] <= limit_threshold * (-1):
                    tracking[j][k][i] -= 1

                # Limit the MXL buffer depending on tracking record
                if tracking[j][k][i] > 0:
                    publish_buffer[j, k][i*2+1] = limit_upper
                elif tracking[j][k][i] < 0:
                    publish_buffer[j, k][i*2+1] = limit_lower

    return publish_buffer


# Takes buffer into byte array and shift array
def buffer_preprocessor(buffer_in):

    for j in range(0, num_of_board):
        for k in range(0, num_of_taxel):

            # Transfer MLX buffer to byte array
            byte_array[j*step+k*6+0] = buffer_in[j, k][1]
            byte_array[j*step+k*6+1] = buffer_in[j, k][2]
            byte_array[j*step+k*6+2] = buffer_in[j, k][3]
            byte_array[j*step+k*6+3] = buffer_in[j, k][4]
            byte_array[j*step+k*6+4] = buffer_in[j, k][5]
            byte_array[j*step+k*6+5] = buffer_in[j, k][6]

            # Put all MSB & LSB in a buffer
            shift_array[j*48+k*3+0] = byte_array[j*step+k*6+0] << 8 | byte_array[j*step+k*6+1]
            shift_array[j*48+k*3+1] = byte_array[j*step+k*6+2] << 8 | byte_array[j*step+k*6+3]
            shift_array[j*48+k*3+2] = byte_array[j*step+k*6+4] << 8 | byte_array[j*step+k*6+5]

    return byte_array, shift_array


# Resetting tracker for the tracking issue
def reset():

    while True:
        raw_input('Press entre to reset')

        for i in range(0, num_of_board):
            for j in range(0, num_of_taxel):
                for k in range(0, num_of_axis):
                   tracking[i][j][k] = 0


# Send pickled buffer via TCP for ROS
def ros_pickle_send(buffer_in):

    #pickle_array = pickle.dumps(buffer_in)
    #conn.sendall(pickle_array)
    conn.send(buffer_in)


# Send unpickled buffer via TCP for Visulisation
def visualisation_send(buffer_in):

    conn.sendall(buffer_in)



###################
## Main Function ##
###################

if __name__ == '__main__':

    # Initialisation process
    csv_log_init()
    csv_debug_processed_init()
    csv_debug_raw_init()
    ntcan_init()
    can_init()

    #start_sensor_all()
    #record_baseline()
    conn = server_init()
    #thread.start_new_thread(reset, ())

    # Taking first MLX readings
    read_sensor_all()
    mlx_buffer_cache()
    mlx_publish = deepcopy(mlx_buffer)

    try:
        while(True):

            # Caching previous buffers
            mlx_previous = deepcopy(mlx_buffer)

            # Reading new buffers
            read_sensor_all()
            mlx_buffer = mlx_buffer_cache()
            mlx_publish = deepcopy(mlx_buffer)

            # Write raw data to CSV for debugging
            #byte_array, shift_array = buffer_preprocessor(mlx_buffer)
            #csv_debug_raw_write(shift_array)

            # Calculate difference between previous and current step reading
            mlx_publish = limitation_handler(mlx_previous, mlx_buffer, mlx_publish)

            # Preprocess MLX publish buffer into byte and shift buffer
            byte_array, shift_array = buffer_preprocessor(mlx_publish)

            # Write processed data to CSV for debugging
            #csv_debug_processed_write(shift_array)

            # Send pickled buffer via TCP for ROS
            ros_pickle_send(shift_array)

            # Send unpickled buffer via TCP for Visulisation
            #visualisation_send(byte_buffer)


    except KeyboardInterrupt:
        print('Stop')
        conn.close()

