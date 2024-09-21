import sys
print("Gesamt_Gradienten.py wird ausgefuehrt...")
import pandas as pd
print("Pandas Version: {}".format(pd.__version__))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt#
print("Matplotlib Version: {}".format(plt.matplotlib.__version__))
import os
import numpy as np
import matplotlib

def calculate_gradients(data):
    """Calculate the gradients and their magnitude, returning the modified DataFrame."""
    data['dx'] = data['Sensor 1: Achse x'] - data['Sensor 2: Achse x']
    data['dy'] = data['Sensor 1: Achse y'] - data['Sensor 2: Achse y']
    data['dz'] = data['Sensor 1: Achse z'] - data['Sensor 2: Achse z']
    
    # Calculate the magnitude of the gradient
    data['Btotal'] = np.sqrt(data['dx']**2 + data['dy']**2 + data['dz']**2)
    
    return data

def save_gradients_to_csv(data, output_file):
    """Save the gradients and their magnitude to a CSV file."""
    gradient_data = data[['Time (s)', 'dx', 'dy', 'dz', 'Btotal']]
    gradient_data.to_csv(output_file, index=False)
    print("Gradients and magnitude saved to {}".format(output_file))

def plot_Betrag(data, output_file, time_column):
    """Plot the magnitude of the gradients and save the plot to a file."""
    fig, ax = plt.subplots(figsize=(10, 6))
    
    ax.clear()  # Clear the plot before updating
    ax.plot(data[time_column], data['Btotal'], label='Total Field (Btotal)', color='b')
    
    # Set fixed y-axis limits
    ax.set_ylim(0, 0.4)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Total Field (Btotal)')
    ax.legend()
    ax.grid(True)

    # Save the plot
    plt.savefig(output_file, bbox_inches='tight')
    plt.close(fig)
    print("Saved plot to {}".format(output_file))

def plot_gradients(data, output_file, time_column, initial_range=(-0.25, 0.25)):
    """Plot the gradients and save the plot to a file."""
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    axes_names = ['x', 'y', 'z']
    
    spans = []
    for axis in axes_names:
        gradient = data['d{}'.format(axis)]
        spans.append(gradient.max() - gradient.min())

    max_span = max(spans)
    y_axis_margin = 0.05 * max_span  # Toleranzmarge

    for i, ax in enumerate(axs):
        gradient = data['d{}'.format(axes_names[i])]
        gradient_min = gradient.min()
        gradient_max = gradient.max()

        # Set the y-limits to ensure a consistent range
        if gradient_min < initial_range[0] or gradient_max > initial_range[1]:
            center = (gradient_max + gradient_min) / 2
            y_axis_min = center - max_span / 2 - y_axis_margin
            y_axis_max = center + max_span / 2 + y_axis_margin
        else:
            y_axis_min, y_axis_max = initial_range

        ax.plot(data[time_column], gradient, label='d{}'.format(axes_names[i]), color='r')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Gradient ({})".format(axes_names[i]))
        ax.set_ylim(y_axis_min, y_axis_max)
        ax.legend()
        ax.grid(True)
    
    plt.tight_layout()
    plt.savefig(output_file, bbox_inches='tight')
    plt.close(fig)
    print("Plot saved to {}".format(output_file))

def process_entire_data(csv_file):
    """Process the entire data."""
    try:
        if os.path.getsize(csv_file) == 0:
            print("CSV file is empty.")
            return
        data = pd.read_csv(csv_file)
    except Exception as e:
        print("Error reading CSV file: {}".format(e))
        return

    # Determine the time column
    if 'Time (ms)' in data.columns:
        time_column = 'Time (ms)'
        data['Time (s)'] = data['Time (ms)'] / 1000  # Convert ms to s for consistency
    elif 'Time (s)' in data.columns:
        time_column = 'Time (s)'
    else:
        print("Time column not found.")
        return

    # Calculate gradients and magnitude
    data = calculate_gradients(data)

    # Dynamically generate paths based on the CSV file name
    base_name = os.path.splitext(os.path.basename(csv_file))[0]
    base_path = os.path.dirname(csv_file)
    output_directory = os.path.join(base_path, base_name, "Gesamt")
    os.makedirs(output_directory, exist_ok=True)  # Ensure the output directory exists

    # Save gradients and magnitude to a CSV file
    output_csv_file = os.path.join(output_directory, "gradients_and_magnitude.csv")
    save_gradients_to_csv(data, output_csv_file)
    
    # Save the plot of gradients to a file
    output_plot_file = os.path.join(output_directory, "Gesamt_Gradienten.png")
    plot_gradients(data, output_plot_file, 'Time (s)')

    # Save the plot of the magnitude to a file
    output_betrag_plot_file = os.path.join(output_directory, "Gesamt_Betrag.png")
    plot_Betrag(data, output_betrag_plot_file, 'Time (s)')

# Check if a file path was provided as an argument
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
    print("Messreihe: {}".format(csv_file))
else:
    print("Kein frischer Input. Plotte die letzte Messreihe...")
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"

process_entire_data(csv_file)
