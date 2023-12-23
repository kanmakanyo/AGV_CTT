#! /usr/bin/env python3

#-----------------------------------------------------------------------------#
# Function/library declaration
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
import socket

rospy.init_node('data_converter')
freq = 20 # Hz
rate = rospy.Rate(freq) # Hz

v = 0000

# Callback function
def callback_dbwsub(msg):
    # Get data adc from PLC (ADC) --> Feedback (sensor)
    global v
    v = int(msg.pose.covariance[27])
    print(v)

HOST = ''        # Symbolic name meaning all available interfaces
PORT = 4141     # Arbitrary non-privileged port
rospy.Subscriber('/dbw_pubsub', PoseWithCovarianceStamped, callback_dbwsub)

s = socket.socket()
s.bind((HOST, PORT))

# print(host , port)
s.listen(5)
conn, addr = s.accept()
# print('Connected by', addr)
while True: 
    V_to_PLC = list(str(v))
    if len(V_to_PLC) == 4:
        V_target_ = Vto_PLC[0] + V_to_PLC[1] + V_to_PLC[2] + V_to_PLC[3]
    elif len(V_to_PLC) == 3:
        V_target_ = str("0") + V_to_PLC[0] + V_to_PLC[1] + V_to_PLC[2]    
    elif len(V_to_PLC) == 2:
        V_target_ = str("00") + V_to_PLC[0] + V_to_PLC[1]
    elif len(V_to_PLC) == 1:
        V_target_ = str("000") + V_to_PLC[0]
    elif len(V_to_PLC) == 0:
        V_target_ = str("00000")

    sendplc = str("@") + V_target_
    sendplc = sendplc.encode()
    conn.sendall(sendplc)

    # try: 
    #     conn.sendall(sendplc)
    # except socket.error:
    #     print("Error Occured.")
    #     s.listen()

    rate.sleep()


# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s: 
#     s.bind((HOST, PORT))
#     print('connected mmmm')
#     s.listen()
#     print('connected mmmm')
#     conn, addr = s.accept()
#     print('connected mmmm')
#     with conn: 
#         print('connected by', addr)
#         while True: 
#             V_to_PLC = list(str(v))
#             if len(V_to_PLC) == 4:
#                 V_target_ = Vto_PLC[0] + V_to_PLC[1] + V_to_PLC[2] + V_to_PLC[3]
#             elif len(V_to_PLC) == 3:
#                 V_target_ = str("0") + V_to_PLC[0] + V_to_PLC[1] + V_to_PLC[2]    
#             elif len(V_to_PLC) == 2:
#                 V_target_ = str("00") + V_to_PLC[0] + V_to_PLC[1]
#             elif len(V_to_PLC) == 1:
#                 V_target_ = str("000") + V_to_PLC[0]
#             elif len(V_to_PLC) == 0:
#                 V_target_ = str("00000")

#             sendplc = str("@") + V_target_
#             sendplc = sendplc.encode()
#             conn.sendall(sendplc)

#             rate.sleep()

# while True:
#     conn, addr = s.accept()
#     # try:
#     #     data = conn.recv(1024)

#     #     if not data: break

#     #     print("Client Says: "+data)
#     #     conn.sendall("Server Says:hi")
#     # except socket.error:
#     #     print("Error Occured.")
#     #     break
#     V_to_PLC = list(str(v))
#     if len(V_to_PLC) == 4:
#         V_target_ = Vto_PLC[0] + V_to_PLC[1] + V_to_PLC[2] + V_to_PLC[3]
#     elif len(V_to_PLC) == 3:
#         V_target_ = str("0") + V_to_PLC[0] + V_to_PLC[1] + V_to_PLC[2]    
#     elif len(V_to_PLC) == 2:
#         V_target_ = str("00") + V_to_PLC[0] + V_to_PLC[1]
#     elif len(V_to_PLC) == 1:
#         V_target_ = str("000") + V_to_PLC[0]
#     elif len(V_to_PLC) == 0:
#         V_target_ = str("00000")

#     sendplc = str("@") + V_target_
#     sendplc = sendplc.encode()
#     s.send(sendplc)

#     rate.sleep()

# conn.close()

# while not rospy.is_shutdown():
    
#     asd = "123"
#     sendplc = str("@") + asd
#     sendplc = sendplc.encode()
#     server.send(sendplc)

#     rate.sleep()

# s.close()

#    import socket  # Import socket module
    
#     port = 50000  # Reserve a port for your service every new transfer wants a new port or you must wait.
#     s = socket.socket()  # Create a socket object
#     host = ""  # Get local machine name
#     s.bind(('localhost', port))  # Bind to the port
#     s.listen(5)  # Now wait for client connection.
    
#     print('Server listening....')
    
#     x = 0
    
#     while True:
#         conn, address = s.accept()  # Establish connection with client.
    
#         while True:
#             try:
#                 print('Got connection from', address)
#                 data = conn.recv(1024)
#                 print('Server received', data)
    
#                 st = 'Thank you for connecting'
#                 byt = st.encode()
#                 conn.send(byt)
    
#                 x += 1
    
#             except Exception as e:
#                 print(e)
#                 break
    
#     conn.close()