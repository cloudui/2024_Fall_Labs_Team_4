from scipy.io import wavfile
import matplotlib.pyplot as plt
import numpy as np
import math

def cross_corr(x, y):
    R = []
    R = np.array(R, dtype=np.int32)
    for m in range(x.size):
        np.append(R, [0], axis=0)
        print(R)
        for n in range(x.size):
            R[m] += (x[n] * y[n-m])
    return R

samplerate1, data1 = wavfile.read('./sound_files/M1.wav')
samplerate2, data2 = wavfile.read('./sound_files/M2.wav')
samplerate3, data3 = wavfile.read('./sound_files/M3.wav')

# square data
data1_square = np.array(data1 ** 2, dtype=np.int32)
data2_square = np.array(data2 ** 2, dtype=np.int32)
data3_square = np.array(data3 ** 2, dtype=np.int32)

# take mean
data1_rms = np.mean(data1_square ** 2)
data2_rms = np.mean(data2_square ** 2)
data3_rms = np.mean(data3_square ** 2)

# take square root
data1_rms = np.sqrt(data1_rms)
data2_rms = np.sqrt(data2_rms)
data3_rms = np.sqrt(data3_rms)

print(data1_rms)
print(data2_rms)
print(data3_rms)

R = cross_corr(data1, data2)
print(R)
