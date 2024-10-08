import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile

samplerate, data = wavfile.read('./baby_shark.wav')
 
# generate spectrogram
plt.specgram(data, Fs=samplerate, cmap="rainbow")
 
# plot
plt.title('Spectrogram of Baby Shark Recorded Audio')
plt.xlabel("Time (s)")
plt.ylabel("Frequency")
plt.show()