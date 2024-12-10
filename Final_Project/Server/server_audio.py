import socket
import struct
import speech_recognition as sr
import socket

# Host IP and port
HOST = '10.5.21.112'  # Replace with your server's IP
PORT = 9500           # Arbitrary non-privileged port (>1024)

RECOGNIZER = sr.Recognizer()
MICROPHONE = sr.Microphone()

def start_server(host, port):
    """Starts a simple server to receive and send data."""
    # Create a socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Bind to the specified host and port
    server_socket.bind((host, port))
    print(f"Server started at {host}:{port}")

    # Listen for incoming connections
    server_socket.listen(5)
    print("Waiting for a connection...")

    try:
        while True:
            # Accept a client connection
            client_socket, client_address = server_socket.accept()
            print(f"Connection established with {client_address}")

            # Receive data from the client
            data = client_socket.recv(1024).decode('utf-8')
            print(f"Received: {data}")

            if data == "direction":
                # Process the received data and send a response
                speech = listen_for_speech()
                command = f'{speech}\n' if speech else "invalid command\n"
                
                client_socket.sendall(command.encode('utf-8'))
                print(f"Sent: {command}")
            elif data == "circle":
                pass
            elif data == "grid":
                pass
            
            # Close the client connection
            client_socket.close()
    except KeyboardInterrupt:
        print("\nShutting down server.")
    finally:
        server_socket.close()

# Function to listen for "left" or "right"
def listen_for_speech():
    with MICROPHONE as source:
        print("Listening for speech...")
        try:
            audio = RECOGNIZER.listen(source, timeout=3, phrase_time_limit=3)
            text = RECOGNIZER.recognize_google(audio).lower()
            print("Finished listening.")
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


if __name__ == "__main__":
    start_server(HOST, PORT)
    # while True:
        # listen_for_speech()