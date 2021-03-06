# Title : uSkin Server
# Date  : 2017/11/19
# Author: Shu


import sys, os, time, types, csv, string
import getopt, util, thread, threading, numpy
import pickle, socket, canopen, ntcan, ipdb
from copy import deepcopy
from shutil import copyfile



###########################
## Globle Configurations ##
###########################

# Board Settings
num_of_taxel    = 16
num_of_axis     = 3
num_of_board    = 5
num_of_tip      = 1
board_start_num = 1

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
difference      = numpy.zeros((num_of_board, num_of_taxel, num_of_axis))
tracking        = numpy.zeros((num_of_board, num_of_taxel, num_of_axis))
limit_threshold = 0xB0
limit_upper     = 0xFA
limit_lower     = 0x0A



##########################
## Function Definitions ##
##########################

# Create a CSV log file for recording baseline
def csv_log_init():

    # Remove old baseline log files
    for i in range(num_of_board):
        del_num = board_start_num + i

        if os.path.exists('visualization/LOG%s.csv' % del_num):
            os.remove('visualization/LOG%s.csv' % del_num)

        ## Create new baseline log files
        #csvfile = open('LOG%s.csv' % del_num, 'wb')
        #filewrite = csv.writer(csvfile)
        #filewrite.writerow('')
        #csvfile.close()


# Create sensor debug logs for debugging purpose
def csv_debug_init():

    # Initialising taxel number header
    tactile_header = []
    for i in range(num_of_board):
        for j in range(num_of_taxel):
            tactile_header.append(str(i)+'B'+str(j)+'X')
            tactile_header.append(str(i)+'B'+str(j)+'Y')
            tactile_header.append(str(i)+'B'+str(j)+'Z')

    if num_of_tip == 1:
        for i in range(num_of_board, num_of_board + num_of_tip):
            for j in range(num_of_taxel / 2):
                tactile_header.append(str(i)+'B'+str(j)+'X')
                tactile_header.append(str(i)+'B'+str(j)+'Y')
                tactile_header.append(str(i)+'B'+str(j)+'Z')

    # Initialising CSV header
    with open('visualization/debug.csv', 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(tactile_header)


# Write to debug log
def csv_debug_write(array):

    # Write to CSV for debugging
    with open('visualization/debug.csv', 'a') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(array)


# NTcan buffer array initialisation
def ntcan_init():

    # cif = ntcan.CIF( net, RxQueueSize, RxTimeOut, TxQueueSize, TxTimeOut, Flags)
    # RxQS, TxQS = 1 for real time application
    net  = 0        # Logical CAN Network [0, 255]
    RxQS = 1        # RxQueueSize [0, 10000]
    RxTO = 2000     # RxTimeOut in Millisconds
    TxQS = 1        # TxQueueSize [0, 10000]
    TxTO = 1000     # TxTimeOut in Millseconds

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
        print "I/O error(%s): " % (errno)


# Read 16 taxel board
def read_sensor_board():

    try:
        for j in range (board_start_num, board_start_num + num_of_board - 1):
            cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)
            for k in range (0, num_of_taxel):
                cmsg_array[j,k].canRead(cif_array[j, k])

    except IOError, (errno):
        print "I/O error(%s): " % (errno)


# Read 8 taxel board
def read_sensor_tip():

    try:
        for j in range (5, 6):
            cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)
            for k in range (0, 8):
                cmsg_array[j,k].canRead(cif_array[j, k])

    except IOError, (errno):
        print "I/O error(%s): " % (errno)


# Transfer all stored CAN message reading into MLX buffer
def mlx_buffer_cache_board():

    for j in range (0, num_of_board):
        for k in range (0, num_of_taxel):
            mlx_buffer[j, k] = cmsg_array[board_start_num+j, k].data.c

    return mlx_buffer


# Transfer all stored CAN message reading into MLX buffer
def mlx_buffer_cache_tip():

    for j in range (5, 6):
        for k in range (0, 8):
            mlx_buffer[j, k] = cmsg_array[j, k].data.c

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
def bit_shift_board(buffer_in):

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

    return bit_buffer


# Combine MSB & LSB into log buffer for 8 taxel board
def bit_shift_tip(buffer_in):

    # Accomodate up to 16 taxels
    x_axis  = {}
    y_axis  = {}
    z_axis  = {}
    bit_buffer = {}

    for j in range (5, 6):
        bit_buffer[j] = list()
        for k in range (0, 8):
            x_axis[j,k] = buffer_in[j,k][1] << 8 | buffer_in[j,k][2]
            y_axis[j,k] = buffer_in[j,k][3] << 8 | buffer_in[j,k][4]
            z_axis[j,k] = buffer_in[j,k][5] << 8 | buffer_in[j,k][6]
            bit_buffer[j].extend([x_axis[j, k], y_axis[j, k], z_axis[j, k]])

    return bit_buffer


# Record baseline
def record_baseline():

    print('Recording baseline...')
    #for i in range (0, 100):
    #    read_sensor_board()
    #    read_sensor_tip()

    for j in range(0, num_of_board - 1):
        write_num = board_start_num + j
        csvfile = open('visualization/LOG%s.csv' % write_num, 'wb')
        filewrite = csv.writer(csvfile)
        time.sleep(1)

        for i in range (0, 100):
            read_sensor_board()
            time.sleep(0.001)
            mlx_buffer_cache_board()
            bit_buffer = bit_shift_board(mlx_buffer)
            filewrite.writerow(bit_buffer[j])

            sys.stdout.write('\rBorad No: %s | Recording progress: %s %%' % (j + 1, i + 1))
            sys.stdout.flush()

        csvfile.close()

    for j in range(num_of_board, num_of_board + num_of_tip):
        write_num = j
        csvfile = open('visualization/LOG%s.csv' % write_num, 'wb' )
        filewrite = csv.writer(csvfile)
        time.sleep(1)

        for i in range (0,100):
            read_sensor_tip()
            time.sleep(0.001)
            mlx_buffer_cache_tip()
            bit_buffer = bit_shift_tip(mlx_buffer)
            filewrite.writerow(bit_buffer[j])

            sys.stdout.write('\rBoard No: %s | Recording progress: %s %%' % (j, i + 1))
            sys.stdout.flush()
    
        csvfile.close()
        
    print('Finished')


def server_init():

    raw_input('Press Enter to start TCP/IP server...')

    # Setup TCP server
    ros = '192.168.11.13'
    local = '127.0.0.1'
    TCP_IP = local
    TCP_PORT = 5007
    BUFFER_SIZE = 20

    # Start TCP server & wait for connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print('Server started, waiting for client conection...')

    conn, addr = s.accept()
    print('Client accepted!')
    print('Press ctrl + c to stop')

    return conn


def limitation_handler(previous_buffer, current_buffer, publish_buffer):

    for j in range(0, num_of_board):
        for k in range(0, num_of_taxel):
            difference[j][k][0] = previous_buffer[j, k][1] - current_buffer[j, k][1]
            difference[j][k][1] = previous_buffer[j, k][3] - current_buffer[j, k][3]
            difference[j][k][2] = previous_buffer[j, k][5] - current_buffer[j, k][5]

            # Handle tracker depending on difference
            for i in range(0, num_of_axis):
                if difference[j][k][i] >= limit_threshold:
                    tracking[j][k][i] += 1
                elif difference[j][k][i] <= limit_threshold * (-1):
                    tracking[j][k][i] -= 1

            # Limit the MXL buffer depending on tracking record
                if tracking[j][k][i] > 0:
                    publish_buffer[j, k][i*2+1] = limit_upper
                elif tracking[j][k][i] < 0:
                    publish_buffer[j, k][i*2+1] = limit_lower

    return publish_buffer


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

    return byte_array

# Send pickled buffer via TCP for ROS
def ros_pickle_send(buffer_in):

    pickle_array = pickle.dumps(buffer_in)
    conn.sendall(buffer)
    time.sleep(0.01)


# Send unpickled buffer via TCP for Visulisation
def visualisation_send(buffer_in):

    conn.sendall(buffer_in)
    time.sleep(0.01)



###################
## Main Function ##
###################

if __name__ == '__main__':

    # Initialisation process
    csv_log_init()
    csv_debug_init()
    ntcan_init()
    can_init()
    record_baseline()
    conn = server_init()

    # Taking first MLX readings
    read_sensor_board()
    read_sensor_tip()
    mlx_buffer_cache_board()
    mlx_buffer_cache_tip()

    try:
        while(True):

            # Caching previous and new MLX readings
            mlx_previous = deepcopy(mlx_buffer)
            read_sensor_board()
            read_sensor_tip()
            mlx_buffer_cache_board()
            mlx_buffer_cache_tip()
            mlx_publish = deepcopy(mlx_buffer)

            # Calculate difference between previous and current step reading
            mlx_publish = limitation_handler(mlx_previous, mlx_buffer, mlx_publish)

            # Preprocess MLX publish buffer into byte and shift buffer
            byte_array = buffer_preprocessor(mlx_publish)

            # Send unpickled buffer via TCP for Visulisation
            visualisation_send(byte_buffer)

            # Send pickled buffer via TCP for ROS
            #ros_pickle_send(shift_array)

            # Write to CSV for debugging
            csv_debug_write(shift_array)

    except KeyboardInterrupt:
        print('Stop')
        conn.close()
