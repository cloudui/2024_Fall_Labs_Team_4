import socket
import speech_recognition as sr
import struct
import time
import cv2
import numpy as np

# Host IP and port
HOST = '10.5.21.112'  # Replace with your server's IP
PORT = 9500           # Arbitrary non-privileged port (>1024)

RECOGNIZER = sr.Recognizer()
MICROPHONE = sr.Microphone()

COLORS = ["red", "red", "green", "blue"]
# hsv thresholds in this order: red, blue, green, other
LOWER = [[0, 100, 100], [160, 100, 100], [35, 50, 100], [100, 100, 100]]
UPPER = [[25, 255, 255], [180, 255, 255], [85, 255, 255], [140, 255, 255]]

CAM = cv2.VideoCapture(0)

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
          command = listen_for_speech()

          if command:
            client_socket.sendall(command.encode('utf-8'))
            print(f"Sent audio command: {command}")
          else:
            client_socket.sendall("invalid command\n".encode('utf-8'))
            print("Sent: invalid command")
        elif data == "circle":
          color = detect_color()
          if color:
            client_socket.sendall(f"{color}\n".encode('utf-8'))
            print(f"Sent color: {color}")
          else:
            client_socket.sendall("invalid color\n".encode('utf-8'))
            print("Sent: invalid color")
          
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
        
def detect_color():

  #image = ep_CAMera.read_cv2_image(strategy="newest", timeout=0.5)
  result, image = CAM.read()
  cv2.imshow('Image Viewer', image)

  color_areas = []

  if result:
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # write image to images folder with random name
    # cv2.imwrite(f"images/{time.time()}.jpg", image)

    for i in range(len(COLORS)):
      mask = cv2.inRange(image_hsv, np.array(LOWER[i]), np.array(UPPER[i]))
      contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

      largest = 0
      for c in contours:
          a = cv2.contourArea(c)
          if a > largest and a > 40_000:
              largest = a

      color_areas.append(largest)

    max_index = 0
    for i in range(len(COLORS)):
      if color_areas[i] > color_areas[max_index]:
        max_index = i

    print("Color detected: " + COLORS[max_index])

    return COLORS[max_index]

  return None


if __name__ == "__main__":
    # start_server(HOST, PORT)
    time.sleep(2)
    while True:
      detect_color()

      key = cv2.waitKey(2000) & 0xFF
      if key == ord('q'):
        break
    
    CAM.release()
    cv2.destroyAllWindows()
      
