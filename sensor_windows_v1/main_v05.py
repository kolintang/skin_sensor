
import sys, os, time, types, csv, string
import getopt, util, thread, threading
import pickle, socket, canopen, ntcan

# Configurations
num_taxel= 16       # Separate taxel and chip, maybe can be useful later
num_of_chip = 16    # Total number of chip per module
num_of_board = 3    # Total number of patch used
num_axis = 3        # X, Y, Z axis
board_start_num = 2 # MTB Board ID


# cif = ntcan.CIF(net, RxQueueSize, RxTimeOut, TxQueueSize, TxTimeOut, Flags)
# RxQS, TxQS = 1 for real time application

net=0           # logical CAN Network [0, 255]
RxQS=1          # RxQueueSize [0, 10000]
RxTO=2000       # RxTimeOut in Millisconds
TxQS=1          # TxQueueSize [0, 10000]
TxTO=1000       # TxTimeOut in Millseconds


# Create cif using a loop
cif_array   = {} #Number of CAN ID or equal to taxel
cmsg_array  = {}

for j in range(board_start_num, board_start_num + num_of_board):
    for k in range(0, num_taxel):
        cif_array[j,k] = ntcan.CIF(net,RxQS)


# CAN-API-Description & set baudrate 0 = 1MBaud
for j in range(board_start_num, board_start_num + num_of_board):
    for k in range(0, num_taxel):
        cif_array[j,k].baudrate = 0


# Create CAN-Message structure
for j in range(board_start_num, board_start_num + num_of_board):
    for k in range(0, num_taxel):
        cmsg_array[j,k] = ntcan.CMSG()


# Target ID of the MTB & trigger the MTB to return the data   
# canWrite...(cif, can-id, len, data ...)
id_base = 0x200
for j in range (board_start_num, board_start_num + num_of_board): 
	cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)            


# Generate CAN address
CAN_address = []
CAN_temp = []
headerID = 0x7

for j in range (board_start_num, board_start_num + num_of_board): 
    for k in range (0, num_taxel):
        CAN_temp.append(headerID << 8 | j  << 4 | k)		
    CAN_address.append(CAN_temp)
    CAN_temp = []

for j in range (board_start_num, num_of_board+board_start_num):
    for k in range (0, num_taxel):
        cif_array[j,k].canIdAdd(CAN_address[j-board_start_num][k])


# Terminal output message
print (cif_array[board_start_num,0])
print (('cmsg lost: %d') % (cmsg_array[board_start_num,0].msg_lost))
print (('cmsg2 %s') % (cmsg_array[board_start_num,0]))
print ('Setup is completed')
raw_input("Press Enter to start...")
print ('Press ctrl + c to stop')


# Setup TCP server
TCP_IP = '192.168.11.11'
TCP_PORT = 5080
BUFFER_SIZE = 8192
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
conn, addr = s.accept()


# Prepare a buffer for TCP communication
data_length = 6  # (X Y Z) * (MSB & LSB)
size = num_of_board * num_taxel * data_length 
step = num_taxel * data_length
byte_array = bytearray(size)
byte_buffer = buffer(byte_array, 0, size)
shift_array = [None] * num_of_board * num_taxel * num_axis

try:
    while(True):
        # Receive can message
        try:
            for j in range (board_start_num, num_of_board+board_start_num):
                cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)
                for k in range (0, num_taxel):
                    cmsg_array[j,k].canRead(cif_array[j,k])
        
        except IOError, (errno):
                print "I/O error(%s): " % (errno)

        # Create buffer to accomodate up 16 taxels
        mlx_buffer = {} 
        for j in range (0, num_of_board):
            for k in range (0, num_taxel):
                mlx_buffer[j,k] = cmsg_array[board_start_num+j,k].data.c
        
        # Put all MSB & LSB in a buffer
        for j in range (0, num_of_board):
            for k in range (0, num_taxel):
                byte_array[j*step + k*6 + 0] = mlx_buffer[j,k][1]
                byte_array[j*step + k*6 + 1] = mlx_buffer[j,k][2]
                byte_array[j*step + k*6 + 2] = mlx_buffer[j,k][3]
                byte_array[j*step + k*6 + 3] = mlx_buffer[j,k][4]
                byte_array[j*step + k*6 + 4] = mlx_buffer[j,k][5]
                byte_array[j*step + k*6 + 5] = mlx_buffer[j,k][6]

        # Put all MSB & LSB in a buffer
        for j in range (0, num_of_board):
            for k in range (0, num_taxel):
                shift_array[j*48+k*3+0] = byte_array[j*step + k*6 + 0] << 8 | byte_array[j*step + k*6 + 1]
                shift_array[j*48+k*3+1] = byte_array[j*step + k*6 + 2] << 8 | byte_array[j*step + k*6 + 3]
                shift_array[j*48+k*3+2] = byte_array[j*step + k*6 + 4] << 8 | byte_array[j*step + k*6 + 5]

        # Send buffer via TCP
        print(shift_array)
        print(len(shift_array))
        pickle_array = pickle.dumps(shift_array)
        conn.sendall(pickle_array)
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print('Stop')
    conn.close()
