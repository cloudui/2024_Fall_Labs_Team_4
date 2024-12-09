import socket
import struct
import speech_recognition as sr

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

# Server loop
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