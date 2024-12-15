import time
from pymavlink import mavutil

# Connect to the Pixhawk via a serial port or UDP
# Replace '/dev/ttyUSB0' with your serial port or use the appropriate connection string for UDP
# For serial connection, use something like '/dev/ttyUSB0' or '/dev/ttyACM0'
# For UDP, use 'udp:127.0.0.1:14550' or your desired UDP endpoint
connection = mavutil.mavlink_connection('/dev/Pixhawk', baud=115200)  # Adjust as needed

# Wait for the first heartbeat to ensure connection
connection.wait_heartbeat()
print("Connected to Pixhawk. Waiting for heartbeat messages...")

# Continuously read messages
while True:
    # Try to get a message
    msg = connection.recv_match(type='HEARTBEAT', blocking=True)
    
    if msg:
        # Print out the heartbeat message
        print(f"Heartbeat received: {msg}")
    
    # You can add a delay if you want to control how often you print
    time.sleep(1)

