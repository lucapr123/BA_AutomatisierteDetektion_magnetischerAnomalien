import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import matplotlib

# Verwende ein nicht-interaktives Backend
matplotlib.use('Agg')

def plot_3d_magnetic_field(csv_file):
    print("Plotting 3D magnetic field visualization with color representing 'dy'...")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Integrated dx")
    ax.set_zlabel("Integrated dz")
    ax.set_title("3D Visualization of Magnetic Field with Colored 'dy'")

    # Read the data from CSV
    try:
        data = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    if data.empty:
        print("Keine Daten zum Plotten.")
        return

    # Extract the data for plotting
    dx = data['Integrated dx']
    dz = data['Integrated dz']
    dy = data['Integrated dy']  # dy values for coloring
    time_values = data["Time (s)"]

    # Create the scatter plot with 'dy' values represented by color
    scatter = ax.scatter(time_values, dx, dz, c=dy, cmap='viridis')

    # Add color bar to indicate the scale of 'dy'
    cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
    cbar.set_label('Integrated dy')

    # Adjust the axes limits to fit the data
    ax.set_xlim(time_values.min(), time_values.max())
    ax.set_ylim(dx.min() - 0.1, dx.max() + 0.1)
    ax.set_zlim(dz.min() - 0.1, dz.max() + 0.1)

    # Save the plot as a PNG file
    output_file = csv_file.replace(".csv", "_3Dplot_colored_dy.png")
    plt.savefig(output_file, bbox_inches='tight')
    print(f"Plot saved to {output_file}")

# Überprüfen, ob ein Argument übergeben wurde
if len(sys.argv) > 1:
    csv_file = f"{sys.argv[1][:-4]}/IntegratedTotalGradients.csv"
    print(f"Verwende den übergebenen Dateipfad: {csv_file}")
else:
    print("Kein Dateipfad übergeben.")
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3/IntegratedTotalGradients.csv"

plot_3d_magnetic_field(csv_file)
