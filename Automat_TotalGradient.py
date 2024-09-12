import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import time
import numpy as np
import os
import sys

def calculate_total_field(data):
    """Calculate the total field (magnitude) from the gradient components."""
    total_field = np.sqrt(data['dx']**2 + data['dy']**2 + data['dz']**2)
    return total_field

def plot_total_field(csv_file, output_file='total_field_plot.png', initial_range=(0.1, 0.3), min_span=0.1):
    """Plots the total field from the CSV file showing only the last 30 seconds and saves the plot as a PNG."""
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    while True:
        try:
            data = pd.read_csv(csv_file)
            
            # Filter data to show only the last 30 seconds
            if 'Time (s)' in data.columns:
                time_column = 'Time (s)'
            else:
                print("Time column not found in the CSV.")
                return

            current_time = data[time_column].max()
            min_time = max(0, current_time - 30)
            filtered_data = data[data[time_column] >= min_time]
            
            # Calculate the total field
            total_field = calculate_total_field(filtered_data)
            
            # Calculate dynamic y-axis limits
            y_min, y_max = total_field.min(), total_field.max()
            span = max(y_max - y_min, min_span)  # Ensure span is at least min_span
            margin = 0.05 * span

            center = (y_max + y_min) / 2
            new_y_min = center - span / 2 - margin
            new_y_max = center + span / 2 + margin

            ax.clear()  # Clear the plot before updating
            
            ax.plot(filtered_data[time_column], total_field, label='Total Field', color='b')
            ax.set_xlim(min_time, current_time)
            # Feste Grenzen setzen
            ax.set_ylim(0, 0.4)  # Set fixed y-axis limits
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Total Field')
            ax.legend()
            ax.grid(True)

            # Save the figure as a PNG file
            plt.savefig(output_file, bbox_inches='tight')
            print(f"Saved plot to {output_file}")

            time.sleep(1)  # Update the plot every second
            
        except Exception as e:
            print(f"Error updating plot: {e}")
            continue

# Überprüfen, ob ein Argument übergeben wurde
if len(sys.argv) > 1:
    csv_file = f"{sys.argv[1][:-4]}/gradients.csv"
    print(f"Verwende den übergebenen Dateipfad: {csv_file}")
else:
    print("Kein Dateipfad übergeben.")
    # Optional: Einen Standardpfad setzen oder das Skript beenden
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"

plot_total_field(csv_file, output_file= f"{csv_file[:-13]}TotalGradient.png", initial_range=(0, 1), min_span=1)
