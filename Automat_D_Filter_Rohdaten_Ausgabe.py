import matplotlib
matplotlib.use('Agg')
import sys
print(sys.argv)
import csv
print(csv)
import matplotlib.pyplot as plt
print(plt)
import time
print(time)
import os
print(os)


# Verwenden des nicht-interaktiven Agg-Backends


def read_csv_file(csv_file):
    """Reads the CSV file and returns the data as a list of dictionaries."""
    with open(csv_file, mode='r') as file:
        reader = csv.DictReader(file)
        data = []
        for row in reader:
            data.append({
                'Time (ms)': int(row['Time (ms)']),
                'Sensor 1: Achse x': float(row['Sensor 1: Achse x']),
                'Sensor 1: Achse y': float(row['Sensor 1: Achse y']),
                'Sensor 1: Achse z': float(row['Sensor 1: Achse z']),
                'Sensor 2: Achse x': float(row['Sensor 2: Achse x']),
                'Sensor 2: Achse y': float(row['Sensor 2: Achse y']),
                'Sensor 2: Achse z': float(row['Sensor 2: Achse z'])
            })
        return data

def plot_live(csv_file, interval=1, output_file='live_plot.png', initial_range=(0, 3), min_span=0.75):
    """Plots live data from the CSV file for each axis (x, y, z) showing only the last 30 seconds and saves the plot as a PNG."""
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    axes_names = ['x', 'y', 'z']
    
    lines = {'sensor1': [], 'sensor2': []}
    
    for i, ax in enumerate(axs):
        lines['sensor1'].append(ax.plot([], [], label=f'Sensor 1: Achse {axes_names[i]}')[0])
        lines['sensor2'].append(ax.plot([], [], label=f'Sensor 2: Achse {axes_names[i]}')[0])
        ax.set_xlim(0, 30)
        ax.set_ylim(initial_range)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(f'Achse {axes_names[i]}')
        ax.legend()
        ax.grid(True)
    
    plt.tight_layout()

    while True:
        try:
            data = read_csv_file(csv_file)
            
            # Convert Time from ms to seconds
            times = [row['Time (ms)'] / 1000 for row in data]
            current_time = max(times)
            min_time = max(0, current_time - 30)
            
            # Filter data to show only the last 30 seconds
            filtered_data = [row for row in data if row['Time (ms)'] / 1000 >= min_time]
            
            spans = []
            for i, axis in enumerate(axes_names):
                y_min = min(row[f'Sensor 1: Achse {axis}'] for row in filtered_data)
                y_max = max(row[f'Sensor 1: Achse {axis}'] for row in filtered_data)
                spans.append(y_max - y_min)
            
            max_span = max(max(spans), min_span)
            margin = 0.05 * max_span
            
            for i, axis in enumerate(axes_names):
                y_min = min(row[f'Sensor 1: Achse {axis}'] for row in filtered_data)
                y_max = max(row[f'Sensor 1: Achse {axis}'] for row in filtered_data)
                
                center = (y_max + y_min) / 2
                new_y_min = center - max_span / 2 - margin
                new_y_max = center + max_span / 2 + margin
                
                lines['sensor1'][i].set_data([row['Time (ms)'] / 1000 for row in filtered_data], 
                                             [row[f'Sensor 1: Achse {axis}'] for row in filtered_data])
                lines['sensor2'][i].set_data([row['Time (ms)'] / 1000 for row in filtered_data], 
                                             [row[f'Sensor 2: Achse {axis}'] for row in filtered_data])
                
                axs[i].set_xlim(min_time, current_time)
                axs[i].set_ylim(new_y_min, new_y_max)
                axs[i].relim()
                axs[i].autoscale_view()
            
            plt.draw()
            plt.pause(interval)
            
            # Sicherstellen, dass das Verzeichnis existiert
            output_dir = os.path.dirname(output_file)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)

            plt.savefig(output_file, bbox_inches='tight')
            print(f"Saved plot to {output_file}")
            
        except Exception as e:
            print(f"Error updating plot: {e}")
            continue

if __name__ == "__main__":
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
        print(f"Verwende den übergebenen Dateipfad: {csv_file}")
    else:
        print("Kein Dateipfad übergeben.")
        csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"

    time.sleep(1)
    print("Starte die Live-Visualisierung...")
    plot_live(csv_file, interval=1, output_file= f"{csv_file[:-4]}/Rohdaten.png")
