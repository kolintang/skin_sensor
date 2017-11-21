
import socket, pickle

# Setup TCP server
TCP_IP = '192.168.11.11'
TCP_PORT = 5080
BUFFER_SIZE = 8192
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

while True:
    data = s.recv(BUFFER_SIZE)
    if not data: break
    data = pickle.loads(data)
    print "received data:", data
