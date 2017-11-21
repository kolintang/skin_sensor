#### main4.py  --> send data from multiple boards over TCP ####
# 21-10-2017 : combining baseline loging & tcp / ip (multiple MTB)

import ntcan  #Importiere Wrapper fuer NTCAN.DLL
import time 
import getopt
import sys
import string
import os
import canopen
import thread
import util
import threading
import types
import time
import csv
import socket
import numpy
from copy import deepcopy

### config ###
num_taxel= 16 #separate taxel and chip, maybe can be useful later
num_of_taxel= 16 #separate taxel and chip, maybe can be useful later
num_of_axis= 16 #separate taxel and chip, maybe can be useful later
num_of_chip = 16  #total number of chip per module
num_of_board = 5
board_start_num = 1 #MTB Board ID

# Limitation initialisation and settings
shift_array = [None] * (num_of_board) * num_of_taxel * num_of_axis
difference = numpy.zeros((num_of_board, num_of_taxel, num_of_axis))
tracking = numpy.zeros((num_of_board, num_of_taxel, num_of_axis))
limit_threshold = 0xA0
limit_upper = 0xFA
limit_lower = 0x0A

##############
### Create a CSV file ####
for i in range(num_of_board):
    del_num = board_start_num + i
    if os.path.exists('visualization/LOG%s.csv' %del_num):
        os.remove('visualization/LOG%s.csv' %del_num)

    csvfile = open('visualization/LOG%s.csv' %del_num,'wb' ) #Create a new csv file
    filewrite = csv.writer(csvfile)
    filewrite.writerow(('B1S1X','B1S1Y','B1S1Z','B1S2X','B1S2Y','B1S2Z','B1S3X','B1S3Y','B1S3Z','B1S4X','B1S4Y','B1S4Z',
                    'B2S1X','B2S1Y','B2S1Z','B2S2X','B2S2Y','B2S2Z','B2S3X','B2S3Y','B2S3Z','B2S4X','B2S4Y','B2S4Z',
                    'B3S1X','B3S1Y','B3S1Z','B3S2X','B3S2Y','B3S2Z','B3S3X','B3S3Y','B3S3Z','B3S4X','B3S4Y','B3S4Z',
                    'B4S1X','B4S1Y','B4S1Z','B4S2X','B4S2Y','B4S2Z','B4S3X','B4S3Y','B4S3Z','B4S4X','B4S4Y','B4S4Z'))
    csvfile.close() # do we need to close it first?

# ---> cif = ntcan.CIF( net, RxQueueSize, RxTimeOut, TxQueueSize, TxTimeOut, Flags)
net=0                                   # logical CAN Network [0, 255]
RxQS=1                                 # RxQueueSize [0, 10000]
RxTO=2000                               # RxTimeOut in Millisconds
TxQS=1                                  # TxQueueSize [0, 10000]
TxTO=1000                               # TxTimeOut in Millseconds

# examples for ntcan ----------------------------------------------------------
#cif1 = ntcan.CIF(net,RxQS,RxTO,TxQS,TxTO)
#cif1 = ntcan.CIF(net,RxQS) #RxQs = 1 --> Queue only 1 value, for real time application

#create cif using a loop
cif_array   = {}#Number of CAN ID or equal to taxel
cmsg_array  = {}

for j in range(board_start_num, num_of_board+board_start_num):
    for k in range(0, num_taxel):
        cif_array[j,k] = ntcan.CIF(net,RxQS)

# validate the configuration & check the CAN-USB availability
print (cif_array[board_start_num,0])
print (cif_array[board_start_num,0].net)
print (cif_array[board_start_num,0].tx_timeout)
print (cif_array[board_start_num,0].rx_timeout)
print (cif_array[board_start_num,0].features)
util.print2lines()

# set baudrate 0 = 1MBaud
# CAN-API-Description

for j in range(board_start_num, num_of_board+board_start_num):
    for k in range(0, num_taxel):
        cif_array[j,k].baudrate = 0
    
# Erzeuge CAN-Messagestruktur
for j in range(board_start_num, num_of_board+board_start_num):
    for k in range(0, num_taxel):
        cmsg_array[j,k] = ntcan.CMSG()

print "cmsg lost: %d"%(cmsg_array[board_start_num,0].msg_lost)
print "cmsg2 %s" %(cmsg_array[board_start_num,0])


id_base = 0x200  #Target ID of the MTB that we want to trigger 
# can write
# canWrite...(cif,can-id,len,data ...)
#cmsg2.canWriteByte(cif2)

for j in range (board_start_num, num_of_board+board_start_num): 
	cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)      	 #Trigger the MTB to return the data               

############################# can read ######################################
#Generate CAN address
CAN_address = []
CAN_temp = []
headerID = 0x7

for j in range (board_start_num, num_of_board+board_start_num): 
    for k in range (0, num_taxel):
        CAN_temp.append(headerID << 8 | j  << 4 | k)		
    CAN_address.append(CAN_temp)
    CAN_temp = []
    
for j in range (board_start_num, num_of_board+board_start_num):
    for k in range (0, num_taxel):
        cif_array[j,k].canIdAdd(CAN_address[j-board_start_num][k]) #add ID

print('Setup is completed')

########################### Baseline recording part ###########
print('Recording a baseline...')

for j in range(0,num_of_board-1):
    write_to = j;
    del_num = board_start_num + j
    csvfile = open('visualization/LOG%s.csv' %del_num,'wb' ) #Create a new csv file
    filewrite = csv.writer(csvfile)
    time.sleep(1)
    for i in range (0,100):
        try:
                for j in range (board_start_num, num_of_board+board_start_num-1):
                    cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)
                    for k in range (0, num_taxel):
                        cmsg_array[j,k].canRead(cif_array[j,k])
                                            
                #print cmsg2 
        except IOError, (errno):
            print "I/O error(%s): " % (errno)

        mlx_buffer = {} #accomodate up 16 taxels
        for j in range (0, num_of_board-1):
            for k in range (0, num_taxel):
                mlx_buffer[j,k] = cmsg_array[board_start_num+j,k].data.c

        #combine MSB | LSB
        x_axis = {} #accomodate up to 16 taxels
        y_axis = {}
        z_axis = {}
        label = {}
        
        #Put all MSB & LSB in a buffer
        for j in range (0, num_of_board-1):
            label[j] = list()
            for k in range (0, num_taxel):
                x_axis[j,k] = mlx_buffer[j,k][1] << 8 | mlx_buffer[j,k][2]
                y_axis[j,k] = mlx_buffer[j,k][3] << 8 | mlx_buffer[j,k][4]
                z_axis[j,k] = mlx_buffer[j,k][5] << 8 | mlx_buffer[j,k][6]

        
        for j in range (0, num_of_board-1):
            for k in range (0, num_taxel):
                label[j].extend([x_axis[j,k], y_axis[j,k], z_axis[j,k]])
               
        #must save according to the label array
        time.sleep(0.01) #delay for 100Hz     
        filewrite.writerow(label[write_to])
        sys.stdout.write('\rRecording progress: %s %%' % (i+1))
        sys.stdout.flush()

    csvfile.close()

### for 2 SDA
for j in range(5,6):
    write_to = j;
    del_num = 5
    csvfile = open('visualization/LOG%s.csv' %del_num,'wb' ) #Create a new csv file
    filewrite = csv.writer(csvfile)
    time.sleep(1)
    for i in range (0,100):
        try:
                for j in range (5,6):
                    cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)
                    for k in range (0, 8):
                        cmsg_array[j,k].canRead(cif_array[j,k])
                                            
                #print cmsg2 
        except IOError, (errno):
            print "I/O error(%s): " % (errno)

        mlx_buffer = {} #accomodate up 16 taxels
        for j in range (5, 6):
            for k in range (0, 8):
                mlx_buffer[j,k] = cmsg_array[j,k].data.c

        #combine MSB | LSB
        x_axis = {} #accomodate up to 16 taxels
        y_axis = {}
        z_axis = {}
        label = {}
        
        #Put all MSB & LSB in a buffer
        for j in range (5, 6):
            label[j] = list()
            for k in range (0,8):
                x_axis[j,k] = mlx_buffer[j,k][1] << 8 | mlx_buffer[j,k][2]
                y_axis[j,k] = mlx_buffer[j,k][3] << 8 | mlx_buffer[j,k][4]
                z_axis[j,k] = mlx_buffer[j,k][5] << 8 | mlx_buffer[j,k][6]

        
        for j in range (5, 6):
            for k in range (0, 8):
                label[j].extend([x_axis[j,k], y_axis[j,k], z_axis[j,k]])
               
        #must save according to the label array
        time.sleep(0.01) #delay for 100Hz     
        filewrite.writerow(label[write_to])
        sys.stdout.write('\rRecording progress: %s %%' % (i+1))
        sys.stdout.flush()

    csvfile.close()
    
print('Finished')


############################### TCP / IP Part ###########
raw_input("Press Enter to start the server...")
print('Press ctrl + c to stop')

### setup TCP server ###

TCP_IP = '127.0.0.1'
TCP_PORT = 5007
BUFFER_SIZE = 20
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)
conn, addr = s.accept()

### Prepare a buffer for TCP communication ###
data_length = 6  # xyz * (MSB & LSB)
size = num_of_board * data_length * num_taxel
step = data_length * num_taxel
test_array = bytearray(size)
test_buffer = buffer(test_array,0, size)


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
            test_array[j*step+k*6+0] = publish_buffer[j, k][1]
            test_array[j*step+k*6+1] = publish_buffer[j, k][2]
            test_array[j*step+k*6+2] = publish_buffer[j, k][3]
            test_array[j*step+k*6+3] = publish_buffer[j, k][4]
            test_array[j*step+k*6+4] = publish_buffer[j, k][5]
            test_array[j*step+k*6+5] = publish_buffer[j, k][6]

            # Put all MSB & LSB in a buffer
            shift_array[j*48+k*3+0] = test_array[j*step+k*6+0] << 8 | test_array[j*step+k*6+1]
            shift_array[j*48+k*3+1] = test_array[j*step+k*6+2] << 8 | test_array[j*step+k*6+3]
            shift_array[j*48+k*3+2] = test_array[j*step+k*6+4] << 8 | test_array[j*step+k*6+5]


#####

try:
    try:
        for j in range (board_start_num, num_of_board+board_start_num-1):
            cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)
            for k in range (0, num_taxel):
                cmsg_array[j,k].canRead(cif_array[j,k])
                
            #print cmsg2 
    except IOError, (errno):
            print "I/O error(%s): " % (errno)

    try:
        for j in range (5, 6):
            cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)
            for k in range (0, 8):
                cmsg_array[j,k].canRead(cif_array[j,k])

    except IOError, (errno):
            print "I/O error(%s): " % (errno)            

    for j in range (0, num_of_board):
        for k in range (0, num_taxel):
            mlx_buffer[j,k] = cmsg_array[board_start_num+j,k].data.c

    while(True):

        mlx_previous = deepcopy(mlx_buffer)

        try:
            for j in range (board_start_num, num_of_board+board_start_num-1):
                cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)
                for k in range (0, num_taxel):
                    cmsg_array[j,k].canRead(cif_array[j,k])
                    
                #print cmsg2 
        except IOError, (errno):
                print "I/O error(%s): " % (errno)

        try:
            for j in range (5, 6):
                cmsg_array[j,0].canWriteByte(cif_array[j,0],(id_base|j),2,7,0)
                for k in range (0, 8):
                    cmsg_array[j,k].canRead(cif_array[j,k])

        except IOError, (errno):
                print "I/O error(%s): " % (errno)            

        for j in range (0, num_of_board):
            for k in range (0, num_taxel):
                mlx_buffer[j,k] = cmsg_array[board_start_num+j,k].data.c
        
        ##Put all MSB & LSB in a buffer
        #for j in range (0, num_of_board):
        #    for k in range (0, num_taxel):
        #        test_array[j*step + k*6] = mlx_buffer[j,k][1]
        #        test_array[j*step + k*6 + 1] = mlx_buffer[j,k][2]
        #        test_array[j*step + k*6 + 2] = mlx_buffer[j,k][3]
        #        test_array[j*step + k*6 + 3] = mlx_buffer[j,k][4]
        #        test_array[j*step + k*6 + 4] = mlx_buffer[j,k][5]
        #        test_array[j*step + k*6 + 5] = mlx_buffer[j,k][6]

        mlx_publish = deepcopy(mlx_buffer)


        # Calculate difference between previous and current step reading
        limitation_handler(mlx_previous, mlx_buffer, mlx_publish)

        #print mlx_buffer[0,0]
        #Send buffer via TCP
        conn.sendall(test_buffer)
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print('Stop')
    conn.close()
    
    #cif2.canIdDelete(0x710)                                 # delete id


# clean up
    #del cif2
    #del cmsg2
