import wave
import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
import time
from scipy.fft import fft, fftfreq
from scipy.signal import butter, filtfilt
from scipy.io.wavfile import write

 # Part 1
def plot_wav_file(file_path):
    with wave.open(file_path, 'r') as wav_file:
        n_channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        framerate = wav_file.getframerate()
        n_frames = wav_file.getnframes()
        duration = n_frames / framerate

        print(f"Number of Channels: {n_channels}")
        print(f"Sample Width (bytes): {sample_width}")
        print(f"Frame Rate (Hz): {framerate}")
        print(f"Number of Frames: {n_frames}")
        print(f"Duration (seconds): {duration}")

        frames = wav_file.readframes(n_frames)
        waveform = np.frombuffer(frames, dtype=np.int16)

        time_axis = np.linspace(0, duration, n_frames)

        plt.figure(figsize=(10, 4))
        plt.plot(time_axis, waveform, label="Mono")
        

        plt.title('Waveform of ' + file_path)
        plt.xlabel('Time [s]')
        plt.ylabel('Amplitude')
        plt.legend()
        plt.show()

file_path = 'Cafe_with_noise.wav'
plot_wav_file(file_path)

# Part 2
def analyze_wav_file(file_path):
    with wave.open(file_path, 'r') as wav_file:
        n_channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        framerate = wav_file.getframerate()
        n_frames = wav_file.getnframes()
        duration = n_frames / framerate

        print(f"Number of Channels: {n_channels}")
        print(f"Sample Width (bytes): {sample_width}")
        print(f"Frame Rate (Hz): {framerate}")
        print(f"Number of Frames: {n_frames}")
        print(f"Duration (seconds): {duration}")

        frames = wav_file.readframes(n_frames)
        waveform = np.frombuffer(frames, dtype=np.int16)

        if n_channels == 2:
            waveform = waveform.reshape(-1, 2).mean(axis=1)

        waveform = waveform / np.max(np.abs(waveform))

        N = len(waveform)
        yf = fft(waveform)
        xf = fftfreq(N, 1 / framerate)

        xf = xf[:N//2]
        yf = np.abs(yf[:N//2])

        plt.figure(figsize=(10, 4))
        plt.plot(xf, yf)
        plt.title('Frequency Domain of ' + file_path)
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Amplitude')
        plt.xlim(0, 8000)  
        plt.grid()
        plt.show()

        voice_freq_indices = (xf >= 300) & (xf <= 3000)
        noise_freq_indices = (xf < 300) | (xf > 3000)

        plt.figure(figsize=(10, 4))
        plt.plot(xf[voice_freq_indices], yf[voice_freq_indices], label='Human Voice Frequencies (300-3000 Hz)')
        plt.plot(xf[noise_freq_indices], yf[noise_freq_indices], label='Noise Frequencies', alpha=0.5)
        plt.title('Frequency Domain with Human Voice and Noise Separated')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Amplitude')
        plt.xlim(0, 8000)
        plt.legend()
        plt.grid()
        plt.show()

file_path = 'Cafe_with_noise.wav'
analyze_wav_file(file_path)\

# Part 3

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    return b, a

def apply_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = filtfilt(b, a, data)
    return y

def filter_wav_file(input_file, output_file, lowcut=300, highcut=1500):
    with wave.open(input_file, 'r') as wav_file:
        n_channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
        framerate = wav_file.getframerate()
        n_frames = wav_file.getnframes()

        frames = wav_file.readframes(n_frames)
        waveform = np.frombuffer(frames, dtype=np.int16)

        if n_channels == 2:
            waveform = waveform.reshape(-1, 2).mean(axis=1)

        waveform = waveform / np.max(np.abs(waveform))

        print("Playing original audio...")
        sd.play(waveform, framerate)
        sd.wait()  

        print("Waiting for 5 seconds...")
        time.sleep(5)

        filtered_waveform = apply_bandpass_filter(waveform, lowcut, highcut, framerate)

        print("Playing filtered audio...")
        sd.play(filtered_waveform, framerate)
        sd.wait()  

        filtered_waveform_int16 = np.int16(filtered_waveform * 32767)
        write(output_file, framerate, filtered_waveform_int16)
        print(f"Filtered audio saved as '{output_file}'.")

input_file = 'Cafe_with_noise.wav'  # Replace this with your actual file path
output_file = 'filtered_voice_output.wav'

filter_wav_file(input_file, output_file, lowcut=300, highcut=1500)