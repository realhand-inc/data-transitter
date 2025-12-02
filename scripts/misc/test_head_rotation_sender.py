import socket
import time
import json
import math

# Server address
SERVER_IP = "192.168.1.56"
SERVER_PORT = 5555
SERVER_ADDR = (SERVER_IP, SERVER_PORT)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Sending head rotation data to {SERVER_IP}:{SERVER_PORT}...")

try:
    start_time = time.time()
    while True:
        # Calculate elapsed time
        elapsed = time.time() - start_time

        # Generate sine and cosine waves
        sin_wave = math.sin(elapsed)
        cos_wave = math.cos(elapsed)

        # Send sine wave, cosine wave, and product
        data_to_send = f"{sin_wave}, {cos_wave}, {sin_wave * cos_wave}"
        message = data_to_send.encode('utf-8')

        # Send the data
        sock.sendto(message, SERVER_ADDR)
        print(f"Sent: {data_to_send}")

        # Wait for a second
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping sender.")
finally:
    sock.close()
    print("Socket closed.")
