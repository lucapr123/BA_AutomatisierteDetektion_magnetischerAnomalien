# Zum Visualisieren der Sensordaten (MCU-gefiltert und zusätzlich MCU-ungefiltert)
# Anstatt Automat_D_Filter_Rohdaten_Ausgabe.py verwenden
import pandas as pd
import matplotlib.pyplot as plt
import time

def plot_live(csv_file, interval=1, output_file='live_plot.png', initial_range=(0, 3), min_span=0.75):
    """Plots live data from the CSV file for each axis (x, y, z) showing only the last 30 seconds and saves the plot as a PNG."""
    plt.ion()  # Turn on interactive mode
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    axes_names = ['x', 'y', 'z']
    
    # Initialize lines for raw and filtered data for each axis
    lines = {
        'raw_sensor1': [],
        'filtered_sensor1': [],
        'raw_sensor2': [],
        'filtered_sensor2': []
    }
    
    for i, ax in enumerate(axs):
        lines['raw_sensor1'].append(ax.plot([], [], label=f'Raw Sensor 1: Achse {axes_names[i]}')[0])
        lines['filtered_sensor1'].append(ax.plot([], [], label=f'Filtered Sensor 1: Achse {axes_names[i]}')[0])
        lines['raw_sensor2'].append(ax.plot([], [], label=f'Raw Sensor 2: Achse {axes_names[i]}')[0])
        lines['filtered_sensor2'].append(ax.plot([], [], label=f'Filtered Sensor 2: Achse {axes_names[i]}')[0])
        ax.set_xlim(0, 30)  # Set x-axis limits to show the last 30 seconds
        ax.set_ylim(initial_range)  # Set initial y-axis limits
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(f'Achse {axes_names[i]}')
        ax.legend()
        ax.grid(True)
    
    plt.tight_layout()
    
    while True:
        try:
            data = pd.read_csv(csv_file)
            
            # Convert Time from ms to seconds
            data['Time (s)'] = data['Time (ms)'] / 1000
            
            # Filter data to show only the last 30 seconds
            current_time = data['Time (s)'].max()
            min_time = max(0, current_time - 30)
            filtered_data = data[data['Time (s)'] >= min_time]
            
            # Calculate the y-axis range (span) for each axis
            spans = []
            for i, axis in enumerate(axes_names):
                y_min, y_max = float('inf'), float('-inf')
                for sensor_type in ['Raw', 'Filtered']:
                    sensor_min = filtered_data[f'{sensor_type} Sensor 1: Achse {axis}'].min()
                    sensor_max = filtered_data[f'{sensor_type} Sensor 1: Achse {axis}'].max()
                    sensor_min_2 = filtered_data[f'{sensor_type} Sensor 2: Achse {axis}'].min()
                    sensor_max_2 = filtered_data[f'{sensor_type} Sensor 2: Achse {axis}'].max()
                    
                    y_min = min(y_min, sensor_min, sensor_min_2)
                    y_max = max(y_max, sensor_max, sensor_max_2)
                
                span = y_max - y_min
                spans.append(span)
            
            # Determine the maximum span to ensure all subplots have the same span
            max_span = max(max(spans), min_span)  # Ensure the span is at least min_span (e.g., 1)
            margin = 0.05 * max_span
            
            for i, axis in enumerate(axes_names):
                # Get the current y-limits
                y_min, y_max = float('inf'), float('-inf')
                for sensor_type in ['Raw', 'Filtered']:
                    sensor_min = filtered_data[f'{sensor_type} Sensor 1: Achse {axis}'].min()
                    sensor_max = filtered_data[f'{sensor_type} Sensor 1: Achse {axis}'].max()
                    sensor_min_2 = filtered_data[f'{sensor_type} Sensor 2: Achse {axis}'].min()
                    sensor_max_2 = filtered_data[f'{sensor_type} Sensor 2: Achse {axis}'].max()
                    
                    y_min = min(y_min, sensor_min, sensor_min_2)
                    y_max = max(y_max, sensor_max, sensor_max_2)
                
                center = (y_max + y_min) / 2
                new_y_min = center - max_span / 2 - margin
                new_y_max = center + max_span / 2 + margin
                
                # Update raw and filtered lines for Sensor 1
                lines['raw_sensor1'][i].set_data(filtered_data['Time (s)'], filtered_data[f'Raw Sensor 1: Achse {axis}'])
                lines['filtered_sensor1'][i].set_data(filtered_data['Time (s)'], filtered_data[f'Filtered Sensor 1: Achse {axis}'])
                
                # Update raw and filtered lines for Sensor 2
                lines['raw_sensor2'][i].set_data(filtered_data['Time (s)'], filtered_data[f'Raw Sensor 2: Achse {axis}'])
                lines['filtered_sensor2'][i].set_data(filtered_data['Time (s)'], filtered_data[f'Filtered Sensor 2: Achse {axis}'])
                
                # Set dynamic y-axis limits based on calculated range
                axs[i].set_xlim(min_time, current_time)
                axs[i].set_ylim(new_y_min, new_y_max)
                axs[i].relim()
                axs[i].autoscale_view()
            
            plt.draw()
            plt.pause(interval)
            
            # Save the figure as a PNG file
            plt.savefig(output_file, bbox_inches='tight')
            print(f"Saved plot to {output_file}")
            
        except Exception as e:
            print(f"Error updating plot: {e}")
            break

    plt.ioff()
    plt.show()

# Example usage:
csv_file = '/home/gf250/Dokumente/Visualisierung/2908_Launch/Messung_1.csv'  # Replace with your actual CSV file path
plot_live(csv_file, interval=1, output_file= f"{csv_file[:-4]}/Rohdaten.png")
