import numpy as np
import matplotlib.pyplot as plt

def read_and_plot_first_column(file_path):
    # Read the file and extract the first column
    data = np.loadtxt(file_path)
    first_column = data[:, 3]

    # Plot the first column
    plt.figure()
    plt.plot(first_column, label='First Column')
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('Plot of the First Column')
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage
file_path = '/path/to/your/trajectory_2024-5-20-22-7.txt'  # Update with the actual file path
read_and_plot_first_column(file_path)