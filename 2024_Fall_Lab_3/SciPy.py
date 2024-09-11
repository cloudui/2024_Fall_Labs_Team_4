import numpy as np 
import matplotlib.pyplot as plt
from scipy import linalg 
from scipy.optimize import minimize
from scipy.fft import fft, fftfreq

  
# Part 1
a = np.array([[3, 1], [1, 2]]) 
b = np.array([9, 8]) 
  
# Solving the linear equations 
ans = linalg.solve(a, b)
print("Part 1 Answer:") 
print(ans)

# Part 2
def func(x):
    return x**2 + 2*x

# Initial guess for x
x0 = 0.0

# Use SciPy's minimize function
result = minimize(func, x0)
print("Part 2 Answer:")
print(f"Minimum occurs at x = {result.x[0]}")
print(f"Minimum value of the function is {result.fun}")

#Part 3

def f(x):
    return np.sin(100 * np.pi * x) + 0.5 * np.sin(160 * np.pi * x)

# Sample space and generated signal
N = 1000           
T = 1.0 / 1000.0  
x = np.linspace(0.0, N*T, N, endpoint=False)
y = f(x)

# Fourier transform using SciPy
yf = fft(y)
xf = fftfreq(N, T)[:N//2]  
# Plot
print("Part 3 Answer: (plot incoming...)")
plt.figure(figsize=(10, 6))
plt.plot(xf, 2.0/N * np.abs(yf[:N//2]))
plt.grid()
plt.title('Frequency Response of f(x)')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Amplitude')
plt.show()


