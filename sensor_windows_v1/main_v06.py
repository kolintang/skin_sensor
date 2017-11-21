
import sys, os, time, types, csv, string
import getopt, util, thread, threading
import pickle, socket, canopen, ntcan
from shutil import copyfile

# Configurations
num_of_taxel= 16       # Separate taxel and chip, maybe can be useful later
num_of_chip = 16    # Total number of chip per module
num_of_board = 1    # Total number of patch used
num_of_axis = 3        # X, Y, Z axis
board_start_num = 9 # MTB Board ID


# Create a CSV log file for recording baseline
for i in range (num_of_board):
    del_num = board_start_num + i
    if os.path.exists('LOG%s.csv' % (del_num)):
        os.remove('LOG%s.csv' % (del_num))

    csvfile = open ('LOG%s.csv' % (del_num), 'wb')
    write = csv.writer(csvfile)
    write.writerow(('B1S1X', 'B1S1Y', 'B1S1Z', 'B1S2X', 'B1S2Y', 'B1S2Z',
                    'B1S3X', 'B1S3Y', 'B1S3Z', 'B1S4X', 'B1S4Y', 'B1S4Z',
                    'B2S1X', 'B2S1Y', 'B2S1Z', 'B2S2X', 'B2S2Y', 'B2S2Z',
                    'B2S3X', 'B2S3Y', 'B2S3Z', 'B2S4X', 'B2S4Y', 'B2S4Z',
                    'B3S1X', 'B3S1Y', 'B3S1Z', 'B3S2X', 'B3S2Y', 'B3S2Z',
                    'B3S3X', 'B3S3Y', 'B3S3Z', 'B3S4X', 'B3S4Y', 'B3S4Z',
                    'B4S1X', 'B4S1Y', 'B4S1Z', 'B4S2X', 'B4S2Y', 'B4S2Z',
                    'B4S3X', 'B4S3Y', 'B4S3Z', 'B4S4X', 'B4S4Y', 'B4S4Z'))
    csvfile.close()

# Create sensor debug logs for debug purpose
csv_name = './debug.csv'
tactile_header = []

def csv_init():
    # Initialising taxel number header
    for i in range (num_of_board):
        for j in range (num_of_taxel):
            tactile_header.append(str(i)+'B'+str(j)+'X')
            tactile_header.append(str(i)+'B'+str(j)+'Y')
            tactile_header.append(str(i)+'B'+str(j)+'Z')

    # Initialising CSV header
    with open(csv_name, 'w') as csvfile:
        writer = csv.writer(csvfile, lineterminator='\n')
        writer.writerow(tactile_header)


# cif = ntcan.CIF(net, RxQueueSize, RxTimeOut, TxQueueSize, TxTimeOut, Flags)
# RxQS, TxQS = 1 for real time application
net  = 0        # Logical CAN Network [0, 255]
RxQS = 1        # RxQueueSize [0, 10000]
RxTO = 2000     # RxTimeOut in Millisconds
TxQS = 1        # TxQueueSize [0, 10000]
TxTO = 1000     # TxTimeOut in Millseconds


# Create cif using a loop
cif_array   = {}    # Number of CAN ID or equal to taxel
cmsg_array  = {}

for j in range (board_start_num, board_start_num + num_of_board):
    for k in range (0, num_of_taxel):
        cif_array[j, k] = ntcan.CIF(net, RxQS)

# CAN-API-Description & set baudrate 0 = 1MBaud
for j in range (board_start_num, board_start_num + num_of_board):
    for k in range (0, num_of_taxel):
        cif_array[j, k].baudrate = 0

# Create CAN-Message structure
for j in range (board_start_num, board_start_num + num_of_board):
    for k in range (0, num_of_taxel):
        cmsg_array[j, k] = ntcan.CMSG()

# Target ID of the MTB & trigger the MTB to return the data   
# canWrite...(cif, can-id, len, data ...)
id_base = 0x200
for j in range (board_start_num, board_start_num + num_of_board): 
	cmsg_array[j, 0].canWriteByte(cif_array[j, 0],(id_base | j), 2, 7, 0)            


# Generate CAN address
CAN_addr = []
CAN_temp = []
headerID = 0x7

for j in range (board_start_num, board_start_num + num_of_board): 
    for k in range (0, num_of_taxel):
        CAN_temp.append(headerID << 8 | j << 4 | k)		
    CAN_addr.append(CAN_temp)
    CAN_temp = []

for j in range (board_start_num, board_start_num + num_of_board):
    for k in range (0, num_of_taxel):
        cif_array[j, k].canIdAdd(CAN_addr[j-board_start_num][k])


# Terminal output message
print (cif_array[board_start_num, 0])
print (('cmsg lost: %d') % (cmsg_array[board_start_num, 0].msg_lost))
print (('cmsg2 %s') % (cmsg_array[board_start_num, 0]))
print ('Setup is completed')


# Record baseline
print ('Recording baseline...')
for j in range (0, num_of_board):
    write_to = j
    del_num = board_start_num + j
    csvfile = open ('LOG%s.csv' % (del_num), 'wb' )
    filewrite = csv.writer(csvfile) 
    for i in range (0, 100):
        try:
            for j in range (board_start_num, board_start_num + num_of_board):
                cmsg_array[j, 0].canWriteByte(cif_array[j, 0], (id_base | j), 2, 7, 0)
                for k in range (0, num_of_taxel):
                    cmsg_array[j, k].canRead(cif_array[j, k])
                                            
        except IOError, (errno):
            print "I/O error(%s): " % (errno)

        mlx_buffer = {} # Accomodate up 16 taxels
        for j in range (0, num_of_board):
            for k in range (0, num_of_taxel):
                mlx_buffer[j, k] = cmsg_array[board_start_num + j, k].data.c

        # Combine MSB | LSB
        x_axis = {} # Accomodate up to 16 taxels
        y_axis = {}
        z_axis = {}
        label  = {}
        
        #Put all MSB & LSB in a buffer
        for j in range (0, num_of_board):
            label[j] = list()
            for k in range (0, num_of_taxel):
                x_axis[j, k] = mlx_buffer[j, k][1] << 8 | mlx_buffer[j, k][2]
                y_axis[j, k] = mlx_buffer[j, k][3] << 8 | mlx_buffer[j, k][4]
                z_axis[j, k] = mlx_buffer[j, k][5] << 8 | mlx_buffer[j, k][6]
        
        for j in range (0, num_of_board):
            for k in range (0, num_of_taxel):
                label[j].extend([x_axis[j, k], y_axis[j, k], z_axis[j, k]])
               
        # Must save according to the label array
        filewrite.writerow(label[write_to])
    csvfile.close()

# Create Log1 and Log2 just for visualisation program
copyfile("./LOG9.csv", "./LOG1.csv")
copyfile("./LOG9.csv", "./LOG2.csv")

print('Finished')
raw_input("Press Enter to start...")
print ('Press ctrl + c to stop')

# Setup TCP server
#TCP_IP = '192.168.11.11'
TCP_IP = '127.0.0.1'
TCP_PORT = 5007
BUFFER_SIZE = 8192
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
conn, addr = s.accept()

# Prepare a buffer for TCP communication
data_length = 6  # (X Y Z) * (MSB & LSB)
size = num_of_board * num_of_taxel * data_length 
step = num_of_taxel * data_length
byte_array = bytearray(size)
byte_buffer = buffer(byte_array, 0, size)
shift_array = [None] * num_of_board * num_of_taxel * num_of_axis


try:
    csv_init()
    while(True):
        # Receive can message
        try:
            for j in range (board_start_num, num_of_board+board_start_num):
                cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)
                for k in range (0, num_of_taxel):
                    cmsg_array[j,k].canRead(cif_array[j,k])
        
        except IOError, (errno):
                print "I/O error(%s): " % (errno)

        # Create buffer to accomodate up 16 taxels
        mlx_buffer = {} 
        for j in range (0, num_of_board):
            for k in range (0, num_of_taxel):
                mlx_buffer[j,k] = cmsg_array[board_start_num+j,k].data.c
        
        # Put all MSB & LSB in a buffer
        for j in range (0, num_of_board):
            for k in range (0, num_of_taxel):
                byte_array[j*step + k*6 + 0] = mlx_buffer[j,k][1]
                byte_array[j*step + k*6 + 1] = mlx_buffer[j,k][2]
                byte_array[j*step + k*6 + 2] = mlx_buffer[j,k][3]
                byte_array[j*step + k*6 + 3] = mlx_buffer[j,k][4]
                byte_array[j*step + k*6 + 4] = mlx_buffer[j,k][5]
                byte_array[j*step + k*6 + 5] = mlx_buffer[j,k][6]

        # Put all MSB & LSB in a buffer
        for j in range (0, num_of_board):
            for k in range (0, num_of_taxel):
                shift_array[j*48+k*3+0] = byte_array[j*step + k*6 + 0] << 8 | byte_array[j*step + k*6 + 1]
                shift_array[j*48+k*3+1] = byte_array[j*step + k*6 + 2] << 8 | byte_array[j*step + k*6 + 3]
                shift_array[j*48+k*3+2] = byte_array[j*step + k*6 + 4] << 8 | byte_array[j*step + k*6 + 5]

        with open(csv_name, 'a') as csvfile:
            writer = csv.writer(csvfile, lineterminator='\n')
            writer.writerow(shift_array)

        # Send pickled buffer via TCP for ROS
        #print(shift_array)
        #print(len(shift_array))
        #pickle_array = pickle.dumps(shift_array)
        #conn.sendall(pickle_array)
        #time.sleep(0.01)

        # Send unpickled buffer via TCP for Visulisation
        conn.sendall(byte_buffer)
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print('Stop')
    conn.close()
