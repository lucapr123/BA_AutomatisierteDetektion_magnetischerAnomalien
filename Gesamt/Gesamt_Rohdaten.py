import sys
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

def plot_static(csv_file, output_file='plot.png', initial_range=(0.8, 2.2), min_span=0.4):
    """Plots the entire data from the CSV file for each axis (x, y, z) and saves the plot as a PNG."""
    data = pd.read_csv(csv_file)
    
    # Convert Time from ms to seconds
    data['Time (s)'] = data['Time (ms)'] / 1000
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    axes_names = ['x', 'y', 'z']
    
    for i, ax in enumerate(axs):
        ax.plot(data['Time (s)'], data[f'Sensor 1: Achse {axes_names[i]}'], label=f'Sensor 1: Achse {axes_names[i]}')
        ax.plot(data['Time (s)'], data[f'Sensor 2: Achse {axes_names[i]}'], label=f'Sensor 2: Achse {axes_names[i]}')
        ax.set_xlim(data['Time (s)'].min(), data['Time (s)'].max())
        
        # Calculate dynamic y-axis range
        y_min = min(data[f'Sensor 1: Achse {axes_names[i]}'].min(), data[f'Sensor 2: Achse {axes_names[i]}'].min())
        y_max = max(data[f'Sensor 1: Achse {axes_names[i]}'].max(), data[f'Sensor 2: Achse {axes_names[i]}'].max())
        
        span = max(y_max - y_min, min_span)  # Ensure minimum span
        center = (y_max + y_min) / 2
        ax.set_ylim(center - span / 2, center + span / 2)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(f'Achse {axes_names[i]}')
        ax.legend()
        ax.grid(True)
    
    plt.tight_layout()
    
    # Save the figure as a PNG file
    plt.savefig(output_file, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved plot to {output_file}")

# Check if a file path is provided as an argument
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
    print(f"Messreihe: {csv_file}")
else:
    print("Kein frischer Input. Plotte die letzte Messreihe...")
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"

# Extract the base name of the CSV file (e.g., "Messung_6" from "Messung_6.csv")
base_name = os.path.splitext(os.path.basename(csv_file))[0]

# Construct the output file path
base_path = os.path.dirname(csv_file)
output_directory = os.path.join(base_path, base_name, "Gesamt")
output_file = os.path.join(output_directory, "Gesamt_Rohdaten.png")

plot_static(csv_file, output_file=output_file)
