from scipy.io import wavfile
import matplotlib.pyplot as plt
import numpy as np
import math

samplerate1, data1 = wavfile.read('./Sound_Files/M1.wav')
samplerate2, data2 = wavfile.read('./Sound_Files/M2.wav')
samplerate3, data3 = wavfile.read('./Sound_Files/M3.wav')

# plt.plot(data2)
# plt.show()

# square data
data1_rms = data1 ** 2
data2_rms = data2 ** 2
data3_rms = data3 ** 2

# take mean
data1_rms = np.mean(data1_rms)
data2_rms = np.mean(data2_rms)
data3_rms = np.mean(data3_rms)

# take square root
data1_rms = np.sqrt(data1_rms)
data2_rms = np.sqrt(data2_rms)
data3_rms = np.sqrt(data3_rms)

print(data1_rms)
print(data2_rms)
print(data3_rms)