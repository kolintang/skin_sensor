
import socket, sys, pickle, time

TCP_IP = '192.168.11.11'
TCP_PORT = 5089
BUFFER_SIZE = 8192

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((TCP_IP, TCP_PORT))
sock.listen(1)

try:
    while True:
        # Wait for a connection
        print >>sys.stderr, 'waiting for a connection'
        connection, client_address = sock.accept()
        print >>sys.stderr, 'connection from', client_address
        for i in range(50): 
            shift_array = [i]*144
            pickle_array = pickle.dumps(shift_array)
            connection.sendall(pickle_array)
            print >>sys.stderr, 'Send', pickle_array
            time.sleep(0.01)

except KeyboardInterrupt:
    connection.close
    print('Stop')

