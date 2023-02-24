import socket
import struct

# Set up the IP address and port number of the MR1000 sensor
ip_address = "192.168.1.10"
port = 2111

# Create a socket object and connect to the MR1000 sensor
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ip_address, port))

# Send a request to the sensor to start data acquisition
command = "sMN LMDscandata 1\n"
sock.sendall(command.encode())

# Receive the response from the sensor
response = sock.recv(1206)

# Extract the range data from the response
range_data = struct.unpack(">541H", response[38:1080])

# Print the range data
print(range_data)

# Send a request to the sensor to stop data acquisition
command = "sMN LMDscandata 0\n"
sock.sendall(command.encode())

# Close the socket connection
sock.close()
