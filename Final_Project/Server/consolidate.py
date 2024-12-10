import socket
import struct
import speech_recognition as sr
import cv2
import time
import matplotlib.pyplot as plt
import numpy as np

# Host IP and port
HOST = '172.20.10.8'  # Replace with your server's IP
PORT = 9931           # Arbitrary non-privileged port (>1024)

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)
print(f"Server started. Listening on {HOST}:{PORT}")

# Define the format string for the struct
# 'h' = int16_t (sequence), '50s' = char[50] (text message)
format_string = '<h50s'

# Initialize speech recognition components
recognizer = sr.Recognizer()
microphone = sr.Microphone()

# hsv thresholds in this order: red, blue, green, other
lower = [[0, 26, 0], [99, 133, 77], [34, 75, 74]]
upper = [[41, 255, 255], [179, 255, 255], [70, 255, 255]]

colors = ["red", "blue", "green"]

# message
seq = 1
text = b'Hello World'
# Pack data into binary format
packed_data = struct.pack(format_string, seq, text)

# Function to listen for "left" or "right"
def listen_for_speech():
    with microphone as source:
        print("Listening for speech...")
        try:
            audio = recognizer.listen(source, timeout=2, phrase_time_limit=2)
            text = recognizer.recognize_google(audio)
            print(f"Recognized speech: {text}")
            if "left" in text:
                return "left"
            elif "right" in text:
                return "right"
            else:
                return None
        except sr.UnknownValueError:
            print("Google Speech Recognition could not understand audio.")
            return None
        except sr.RequestError as e:
            print(f"Could not request results from Google Speech Recognition service; {e}")
            return None
        except sr.WaitTimeoutError:
            print("Speech recognition timed out.")
            return None
        
def detect_color():
    cam = cv2.VideoCapture(0)

    #image = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    result, image = cam.read()
    time.sleep(1)

    result, image = cam.read()
    time.sleep(1)

    color_areas = []

    if result:
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for i in range(3):
            mask = cv2.inRange(image_hsv, np.array(lower[i]), np.array(upper[i]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            largest = 0
            for c in contours:
                a = cv2.contourArea(c)
                if a > largest:
                    largest = a

            color_areas.append(largest)

        max_index = 0
        for i in range(1, 3):
            if color_areas[i] > color_areas[max_index]:
                max_index = i

        print("Color detected: " + colors[max_index])

        time.sleep(1)
        cv2.destroyAllWindows()

        return colors[max_index]

def detect_color_at_square():
    sent = False
    while not sent:
        connection, client_address = server_socket.accept()
        try:
            print("Connection from", client_address)

            # Receive and unpack data from the client
            data = connection.recv(struct.calcsize(format_string))
            if data:
                print("Data received")
                unpacked_data = struct.unpack(format_string, data)
                seq, client_text = unpacked_data
                print(f"Received from client: seq={seq}, text={client_text.decode('utf-8').strip()}")

                # Detect color
                print("Starting detect color...")
                color = detect_color()

                # Set the default message if no command is recognized
                message = color

                print(message)

                # Prepare the response packet
                print("Starting to prepare response data")
                packed_data = struct.pack(format_string, seq + 1, message.encode('utf-8'))
                print(f"Sending packet: seq={seq}, text={message}")
                connection.sendall(packed_data)  

                if message == recognized_command:
                    sent = True
            else:
                print("No data received from client.")
                break
        except:
            print("Something went wrong")
            sent = True
        finally:
            # Clean up the connection
            connection.close()

detect_color_at_square()
detect_color_at_square()

# audio
sent = False
while not sent:
    connection, client_address = server_socket.accept()
    try:
        print("Connection from", client_address)

        # Receive and unpack data from the client
        data = connection.recv(struct.calcsize(format_string))
        if data:
            print("Data received")
            unpacked_data = struct.unpack(format_string, data)
            seq, client_text = unpacked_data
            print(f"Received from client: seq={seq}, text={client_text.decode('utf-8').strip()}")

            # Wait for a recognized speech command
            print("Starting to listen...")
            recognized_command = listen_for_speech()
            print("here1")

            # Set the default message if no command is recognized
            message = recognized_command if recognized_command else "No verbal commands detected"

            print(message)

            # Prepare the response packet
            print("Starting to prepare response data")
            packed_data = struct.pack(format_string, seq + 1, message.encode('utf-8'))
            print(f"Sending packet: seq={seq}, text={message}")
            connection.sendall(packed_data)  

            if message == recognized_command:
                sent = True
        else:
            print("No data received from client.")
            break
    except:
        print("Something went wrong")
        sent = True
    finally:
        # Clean up the connection
        connection.close()

detect_color_at_square()
detect_color_at_square()
detect_color_at_square()

# color detection

# # Create an instance of the struct data
# seq = 1
# distance = 1000
# voltage = 3.7
# text = b'Hello World'

# # Pack data into binary format
# packed_data = struct.pack(format_string, seq, distance, voltage, text)

# while True:
#     # Wait for a connection
#     connection, client_address = server_socket.accept()
#     try:
#         print("Connection from", client_address)

#         # Receive data from the client
#         data = connection.recv(struct.calcsize(format_string)) # Receive data with a buffer size of the data structure defined in the previous section
#         # print("Set data param.")
#         if data:
#             print("Data received")
#             unpacked_data = struct.unpack(format_string, data)
#             seq, distance, voltage, text = unpacked_data
#             print(f"Received: seq={seq}, distance={distance}, voltage={voltage}, text={text.decode('utf-8').strip()}")

#             # Prepare response
#             print("Starting to prepare response data")
#             response_data = struct.pack(format_string, seq + 1, distance + 100, voltage + 1.0, b"Response from server")
#             connection.sendall(response_data)
#             print("Data being sent")
#         else:
#             break
#     except:
#         print("Something went wrong")
#     finally:
#         # Clean up the connection
#         connection.close()