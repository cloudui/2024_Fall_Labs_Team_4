import socket
import speech_recognition as sr
import io


HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 50007      # Port to listen on


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("Server is listening for connections...")


recognizer = sr.Recognizer()

while True:
    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    audio_data = b""
    while True:
        packet = conn.recv(4096)  
        if not packet:
            break
        audio_data += packet

    # Close the connection
    conn.close()
    print("Received audio data.")

 
    audio_stream = io.BytesIO(audio_data)


    try:
        with sr.AudioFile(audio_stream) as source:
            audio = recognizer.record(source)

        text = recognizer.recognize_google(audio).lower()
        print(f"Recognized Text: {text}")

        if "left" in text:
            print("Command: LEFT")
        elif "right" in text:
            print("Command: RIGHT")
        else:
            print("Command not recognized as 'left' or 'right'")

    except sr.UnknownValueError:
        print("Could not understand audio")
    except sr.RequestError as e:
        print(f"Could not request results; {e}")