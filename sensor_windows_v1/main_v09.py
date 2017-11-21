

import sys, os, time, types, csv, string
import getopt, util, thread, threading, numpy
import pickle, socket, canopen, ntcan, ipdb
from copy import deepcopy
from shutil import copyfile



###########################
## Globle Configurations ##
###########################

# Board Settings
id_base = 0x200
num_of_axis = 3         # X, Y, Z axis
num_of_taxel= 16        # Separate taxel and chip, maybe can be useful later
num_of_board = 1        # Total number of patch used
board_start_num = 1     # MTB Board ID

# Buffer and array initialisation
cif_array   = {}        # NTcan cif array
cmsg_array  = {}        # NTcan cmsg array
mlx_buffer = {}         # MLX reading buffer accomodate up to 16 taxels

# Buffer initialisation for TCP/IP communication
data_length = 6  # (X Y Z) * (MSB & LSB)
size = num_of_board * num_of_taxel * data_length 
step = num_of_taxel * data_length
byte_array = bytearray(size)
byte_buffer = buffer(byte_array, 0, size)
shift_array = [None] * num_of_board * num_of_taxel * num_of_axis

# Limitation initialisation and settings
difference = numpy.zeros((num_of_board, num_of_taxel, num_of_axis))
tracking = numpy.zeros((num_of_board, num_of_taxel, num_of_axis))
limit_threshold = 0xB0
limit_upper = 0xFA
limit_lower = 0x0A



##########################
## Function Definitions ##
##########################

# Create a CSV log file for recording baseline
def csv_log_init():

    # Remove old baseline log file
    tactile_header = []
    for i in range(num_of_board):
        del_num = board_start_num + i
        if os.path.exists('LOG%s.csv' % (del_num)):
            os.remove('LOG%s.csv' % (del_num))

    # Initialising CSV header
    with open('LOG%s.csv' % (del_num), 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow('')


# Create sensor debug logs for debug purpose
def csv_debug_init():

    # Initialising taxel number header
    tactile_header = []
    for i in range(num_of_board):
        for j in range(num_of_taxel):
            tactile_header.append(str(i)+'B'+str(j)+'X')
            tactile_header.append(str(i)+'B'+str(j)+'Y')
            tactile_header.append(str(i)+'B'+str(j)+'Z')

    # Initialising CSV header
    with open('debug.csv', 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(tactile_header)


def csv_debug_write(array):

    # Write to CSV for debugging
    with open('debug.csv', 'a') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(array)


# NTcan buffer array initialisation
def ntcan_init():

    # cif = ntcan.CIF(net, RxQueueSize, RxTimeOut, TxQueueSize, TxTimeOut, Flags)
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
            cif_array[j, k].baudrate = 0

            # Create CAN-Message structure
            cmsg_array[j, k] = ntcan.CMSG()

        # Target ID of the MTB & trigger the MTB to return the data
        # canWrite...(cif, can-id, len, data ...)
        cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)

    print(cif_array[board_start_num, 0])
    print(('cmsg lost: %d') % (cmsg_array[board_start_num, 0].msg_lost))
    print(('cmsg2 %s') % (cmsg_array[board_start_num, 0]))
    print('Setup is completed')


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
            cif_array[j, k].canIdAdd(can_addr[j-board_start_num][k])


# Record baseline
def record_baseline():

    print('Recording baseline...')

    # Accomodate up to 16 taxels
    x_axis = {}
    y_axis = {}
    z_axis = {}
    log_buffer  = {}

    for j in range(0, num_of_board):
        write_num = board_start_num + j
        csvfile = open('LOG%s.csv' % (write_num), 'wb' )
        filewrite = csv.writer(csvfile)

        for i in range(0, 100):
            read_sensor(board_start_num, num_of_board, num_of_taxel)
            #read_sensor(board_start_num + num_of_board + num_of_tip,
            #               num_of_tip, num_of_tip_taxel)

        for i in range(0, 100):
            temp_buffer = read_sensor(board_start_num, num_of_board, num_of_taxel)
            for j in range(0, num_of_board):
                log_buffer[j] = list()
                for k in range(0, num_of_taxel):
                    temp_buffer[j, k] = cmsg_array[board_start_num + j, k].data.c
                    x_axis[j, k] = temp_buffer[j, k][1] << 8 | temp_buffer[j, k][2]
                    y_axis[j, k] = temp_buffer[j, k][3] << 8 | temp_buffer[j, k][4]
                    z_axis[j, k] = temp_buffer[j, k][5] << 8 | temp_buffer[j, k][6]
                    log_buffer[j].extend([x_axis[j, k], y_axis[j, k], z_axis[j, k]])

            filewrite.writerow(log_buffer[j])
        csvfile.close()

    # Create Log1 and Log2 just for visualisation program
    copyfile('./LOG9.csv', './LOG1.csv')
    copyfile('./LOG9.csv', './LOG2.csv')
    print('Finished')


def server_init():

    raw_input('Press Enter to start TCP/IP server...')

    # Setup TCP server
    ros = '192.168.11.13'
    local = '127.0.0.1'
    TCP_IP = local
    TCP_PORT = 5007

    # Start TCP server & wait for connection
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)
    print('Server started, waiting for conection...')

    conn, addr = s.accept()
    print('Client accepted!')
    print('Press ctrl + c to stop')

    return conn


def read_sensor(start_num, board_num, taxel_num):

    # Receive can message
    try:
        for j in range(start_num, start_num + board_num):
            cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base|j), 2, 7, 0)
            for k in range(0, taxel_num):
                cmsg_array[j, k].canRead(cif_array[j, k])

    except IOError, (errno):
        print('I/O error: (%s)' % (errno))

    # Read data and transfer to MXL buffer
    for j in range(0, board_num):
        for k in range(0, taxel_num):
            mlx_buffer[j, k] = cmsg_array[start_num + j, k].data.c

    return mlx_buffer


# Calculate difference between previous and current step reading
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

            # Transfer MLX buffer to byte array
            byte_array[j*step+k*6+0] = publish_buffer[j, k][1]
            byte_array[j*step+k*6+1] = publish_buffer[j, k][2]
            byte_array[j*step+k*6+2] = publish_buffer[j, k][3]
            byte_array[j*step+k*6+3] = publish_buffer[j, k][4]
            byte_array[j*step+k*6+4] = publish_buffer[j, k][5]
            byte_array[j*step+k*6+5] = publish_buffer[j, k][6]

            # Put all MSB & LSB in a buffer
            shift_array[j*48+k*3+0] = byte_array[j*step+k*6+0] << 8 | byte_array[j*step+k*6+1]
            shift_array[j*48+k*3+1] = byte_array[j*step+k*6+2] << 8 | byte_array[j*step+k*6+3]
            shift_array[j*48+k*3+2] = byte_array[j*step+k*6+4] << 8 | byte_array[j*step+k*6+5]


# Send pickled buffer via TCP for ROS
def ros_pickle_send(buffer):

      pickle_array = pickle.dumps(buffer)
      conn.sendall(buffer)
      time.sleep(0.01)


# Send unpickled buffer via TCP for Visulisation
def visualisation_send(buffer):

      conn.sendall(buffer)
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
    mlx_buffer = read_sensor(board_start_num, num_of_board, num_of_taxel)
    conn = server_init()

    # Main loop process
    try:
        while(True):

            mlx_previous = deepcopy(mlx_buffer)
            mlx_buffer = read_sensor(board_start_num, num_of_board, num_of_taxel)
            mlx_publish = deepcopy(mlx_buffer)

            # Calculate difference between previous and current step reading
            limitation_handler(mlx_previous, mlx_buffer, mlx_publish)

            # Write to CSV for debugging
            csv_debug_write(shift_array)

            # Send unpickled buffer via TCP for Visulisation
            visualisation_send(byte_buffer)

            # Send pickled buffer via TCP for ROS
            #ros_pickle_send(shift_array)



    except KeyboardInterrupt:
        print('Stop')
        conn.close()

