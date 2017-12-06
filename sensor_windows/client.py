
import socket, pickle

# Setup TCP server
ros = '192.168.11.13'
local = '127.0.0.1'
TCP_IP = local
TCP_PORT = 5007
BUFFER_SIZE = 8192
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

while True:
    data = s.recv(BUFFER_SIZE)
    if not data: break
    data = pickle.loads(data)
    print "received data:", data
