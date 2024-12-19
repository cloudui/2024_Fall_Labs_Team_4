import socket
import speech_recognition as sr
import struct
import time
import cv2
import numpy as np

from pydub import AudioSegment
from pydub.playback import play
import io

# Host IP and port
HOST = '172.20.10.12'  # Replace with your server's IP
PORT = 9500           # Arbitrary non-privileged port (>1024)

RECOGNIZER = sr.Recognizer()
MICROPHONE = sr.Microphone()

RECOGNIZER.energy_threshold = 200

COLORS = ["red", "red", "green", "blue"]
# hsv thresholds in this order: red, blue, green, other
LOWER = [[0, 100, 100], [160, 100, 100], [35, 100, 100], [100, 50, 100]]
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
            client_socket.sendall("invalid\n".encode('utf-8'))
            print("Sent: invalid")
        elif data == "circle":
          color = detect_color()
          if color:
            client_socket.sendall(f"{color}\n".encode('utf-8'))
            print(f"Sent color: {color}")
          else:
            client_socket.sendall("invalid\n".encode('utf-8'))
            print("Sent: invalid")
          
        elif data == "red" or data == "green" or data == "blue":
          direction = detect_grid_path(data)
          if direction:
            client_socket.sendall(f"{direction}\n".encode('utf-8'))
            print(f"Sent turn: {direction}")

          # Close the client connection
        client_socket.close()
  except KeyboardInterrupt:
      print("\nShutting down server.")
  finally:
      server_socket.close()

def process_audio(audio):
  audio_data = io.BytesIO(audio.get_wav_data())  # Convert to a file-like object

  # Load audio into pydub
  original_audio = AudioSegment.from_wav(audio_data)
  # read suffix file
  suffix_audio = AudioSegment.from_wav("audio/turn.wav")

  # Triple the audio
  combined_audio = original_audio + suffix_audio


  output_audio = io.BytesIO()
  combined_audio.export(output_audio, format="wav")

  # Rewind the BytesIO object to the beginning so that speech_recognition can read it
  output_audio.seek(0)

  # Now, create a new SpeechRecognition audio object from the BytesIO data
  audio = sr.AudioData(output_audio.read(), original_audio.frame_rate, original_audio.sample_width)

  return RECOGNIZER.recognize_google(audio).lower()


# Function to listen for "left" or "right"
def listen_for_speech():
  with MICROPHONE as source:
    print("Listening for speech...")
    try:
      audio = RECOGNIZER.record(source, duration=2)
      # text = process_audio(audio)
      text = RECOGNIZER.recognize_google(audio).lower()

      # # save audio as wv file
      # with open(f"audio/{text}.wav", "wb") as f:
      #   f.write(audio.get_wav_data())

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
  # cv2.imshow('Image Viewer', image)

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

def detect_grid_path(color, image=None):
  if color == "red":
    index = [0, 1]
  elif color == "green":
    index = [2]
  else:
    index = [3]
  

  if image is None:
    result, image = CAM.read()
    image = cv2.rotate(image, cv2.ROTATE_180)

  color_areas = []

  if image is not None or result:
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

  for i in index:
    mask = cv2.inRange(image_hsv, np.array(LOWER[i]), np.array(UPPER[i]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest = 0
    center = None

    for c in contours:
        M = cv2.moments(c)
        if M["m00"] != 0:
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
        a = cv2.contourArea(c)
        if a > largest:
            largest = a
            center = (cx, cy)

    color_areas.append((largest, center))

  if center is None:
    return "invalid"
  center = color_areas[0][1]
  
  if color == "red":
    if color_areas[0][0] > color_areas[1][0]:
      center = color_areas[0][1]
    else:
      center = color_areas[1][1]
  
  cv2.circle(image, center, 5, (0, 0, 255), -1)
  cv2.imshow('Image Viewer', image)
  cv2.waitKey(1000)

  image_width = image.shape[1]
  image_height = image.shape[0]
  # if center is clearly on the left
  if center[1] < 2 * image_height / 3:
    return "straight"
  if center[0] < image_width / 2:
    return "left"
  elif center[0] > image_width / 2:
    return "right"
  
  return "invalid"



if __name__ == "__main__":
  time.sleep(2)
  # while True:
  #   for color in COLORS[1:]:
  #     print(color)
  #     print(detect_grid_path(color))
  # for phrase in LiveSpeech():
  #   print(phrase)
  start_server(HOST, PORT)
    # while True:
    #   detect_color()

    #   key = cv2.waitKey(2000) & 0xFF
    #   if key == ord('q'):
    #     break
    
    # CAM.release()
    # cv2.destroyAllWindows()
      
