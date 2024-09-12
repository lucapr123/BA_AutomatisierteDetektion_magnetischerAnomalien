import sys
import matplotlib
matplotlib.use('Agg')
import pandas as pd
import matplotlib.pyplot as plt
import time

def calculate_gradients(data, time_column):
    """Calculate the gradients and return the modified DataFrame."""
    data['dx'] = data['Sensor 1: Achse x'] - data['Sensor 2: Achse x']
    data['dy'] = data['Sensor 1: Achse y'] - data['Sensor 2: Achse y']
    data['dz'] = data['Sensor 1: Achse z'] - data['Sensor 2: Achse z']
    return data

def save_gradients_to_csv(data, output_file):
    """Save the gradients to a CSV file."""
    gradient_data = data[['Time (s)', 'dx', 'dy', 'dz']]
    gradient_data.to_csv(output_file, index=False)
    print(f"Gradients saved to {output_file}")

def save_plot(data, axs, time_column, output_file, initial_range=(-0.4, 0.4)):
    """Update the plot with new data and save it as a PNG."""
    axes_names = ['x', 'y', 'z']
    
    spans = []
    for axis in axes_names:
        gradient = data[f'd{axis}']
        spans.append(gradient.max() - gradient.min())

    max_span = max(spans)
    y_axis_margin = 0.05 * max_span  # Toleranzmarge

    for i, ax in enumerate(axs):
        gradient = data[f'd{axes_names[i]}']
        gradient_min = gradient.min()
        gradient_max = gradient.max()

        # Set the y-limits to ensure a consistent range
        if gradient_min < initial_range[0] or gradient_max > initial_range[1]:
            center = (gradient_max + gradient_min) / 2
            y_axis_min = center - max_span / 2 - y_axis_margin
            y_axis_max = center + max_span / 2 + y_axis_margin
        else:
            y_axis_min, y_axis_max = initial_range

        ax.clear()  # Clear the axis to update the plot
        ax.plot(data[time_column] / 1000, gradient, label=f'd{axes_names[i]}', color='r')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Gradient ({axes_names[i]})")
        ax.set_ylim(y_axis_min, y_axis_max)
        ax.legend()
        ax.grid(True)
    
    plt.savefig(output_file, bbox_inches='tight')
    print(f"Plot saved to {output_file}")

def process_last_30_seconds(csv_file):
    """Process the last 30 seconds of data."""
    try:
        data = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return None, None

    # Determine the time column
    if 'Time (ms)' in data.columns:
        time_column = 'Time (ms)'
        data['Time (s)'] = data['Time (ms)'] / 1000  # Convert ms to s for consistency
    elif 'Time (s)' in data.columns:
        time_column = 'Time (s)'
    else:
        print("Time column not found.")
        return None, None

    current_time = data[time_column].max()
    min_time = max(0, current_time - 30000)  # Last 30 seconds of data

    latest_data = data[data[time_column] >= min_time]

    if latest_data.empty:
        print("No new data in the last 30 seconds.")
        return None, None

    # Calculate gradients
    latest_data = calculate_gradients(latest_data, time_column)

    # Save gradients to a CSV file
    output_csv_file = f"{csv_file[:-4]}/gradients.csv"
    save_gradients_to_csv(latest_data, output_csv_file)
    
    return latest_data, 'Time (s)'

def run_repeatedly(csv_file, interval=2):
    """Repeatedly process and save data every 'interval' seconds."""
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))  # Create the figure once
    
    while True:
        latest_data, time_column = process_last_30_seconds(csv_file)
        
        if latest_data is not None:
            output_file = f"{csv_file[:-4]}/gradient_plot.png"
            save_plot(latest_data, axs, time_column, output_file)
        
        time.sleep(interval)
        

# Check if a file path was provided as an argument
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
    print(f"Using provided file path: {csv_file}")
else:
    print("No file path provided.")
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"

run_repeatedly(csv_file, interval=1)
