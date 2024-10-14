import socket
import struct

# Host IP and port
HOST = '172.20.10.8'  # use ipconfig to find the IP address of the host (or use 0.0.0.0 to accept connection from any address assigned to the server)
PORT = 9500   # Arbitrary non-privileged port (>1024)

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #AF_INET specifies that we are using the IPv4 family of addresses, and SOCK_STREAM specifies a TCP socket

# Bind the socket to the port
server_address = (HOST, PORT)
server_socket.bind(server_address)

# Listen for incoming connections
server_socket.listen(5) # Handles up to 5 simultaneous connections
print(f"Server started. Listening on {HOST}:{PORT}")

# Define the format string for the struct
# 'h' = int16_t, 'i' = int32_t, 'f' = float, '50s' = char[50] (byte string)
format_string = '<hif50s'

# Create an instance of the struct data
seq = 1
distance = 1000
voltage = 3.7
text = b'Hello World'

# Pack data into binary format
packed_data = struct.pack(format_string, seq, distance, voltage, text)

while True:
    # Wait for a connection
    connection, client_address = server_socket.accept()
    try:
        print("Connection from", client_address)

        # Receive data from the client
        data = connection.recv(struct.calcsize(format_string)) # Receive data with a buffer size of the data structure defined in the previous section
        # print("Set data param.")
        if data:
            print("Data received")
            unpacked_data = struct.unpack(format_string, data)
            seq, distance, voltage, text = unpacked_data
            print(f"Received: seq={seq}, distance={distance}, voltage={voltage}, text={text.decode('utf-8').strip()}")

            # Prepare response
            print("Starting to prepare response data")
            response_data = struct.pack(format_string, seq + 1, distance + 100, voltage + 1.0, b"Response from server")
            connection.sendall(response_data)
            print("Data being sent")
        else:
            break
    except:
        print("Something went wrong")
    finally:
        # Clean up the connection
        connection.close()