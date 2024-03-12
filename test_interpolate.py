import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA

# --- Interpolation ---

def interpolate_nd_point(t_new, y, kind='quadratic'):
    """Interpolates each dimension of an n-dimensional point"""
    num_dimensions = y[0].shape[0]
    f = [interp1d(t, y[:, i], kind=kind) for i in range(num_dimensions)]
    return np.array([fi(t_new) for fi in f])

# --- Example Data (Assume your actual data is loaded) ---

t = np.linspace(0, 1, num=3)  
# p1 = np.array([1, 5, 6, 4, 5, 6])  
# p2 = np.array([4, 3, 5, 1, 8, 2])  
# p3 = np.array([7, 3, 1, 6, 3, 9])  
# y = np.array([p1, p2, p3])

target_inputs = [
    [0,1.047,0,1.047,1.047,0],
    [0,1.047,0,0,0,0],
    [0,0,0,0,0,0]
]
y = np.array(target_inputs)

t = np.linspace(0,1,num=len(target_inputs))


# --- Interpolation ---

t_interp = np.linspace(0, 1, num=50) 
interpolated_points = []
for t_val in t_interp:
    new_point = interpolate_nd_point(t_val, y)
    interpolated_points.append(new_point)

# --- Plotting Individual Dimensions ---
plt.figure(figsize=(8, 6))  # Adjust figure size as needed


num_dimensions = interpolated_points[0].shape[0]  

# Plot original points
original_t_vals = np.linspace(0, 1, num=3)  # Assuming t was used for original points
for i in range(num_dimensions):
    plt.plot(original_t_vals, [point[i] for point in y], 'o', label=f'Original - Dimension {i}')


for i in range(num_dimensions):
    plt.plot(t_interp, [point[i] for point in interpolated_points], label=f'Dimension {i}')

plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Interpolated Values for Each Dimension')
plt.legend()
plt.show()