import os
import sys
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt

def calculate_thresholds(data, mean_values, margin=0.025):
    """Calculate thresholds as mean ± margin."""
    return {
        'dx': (mean_values['dx'] - margin, mean_values['dx'] + margin),
        'dy': (mean_values['dy'] - margin, mean_values['dy'] + margin),
        'dz': (mean_values['dz'] - margin, mean_values['dz'] + margin)
    }

def detect_anomalies(data, thresholds):
    """Detect anomalies based on the given thresholds."""
    anomalies = []
    in_anomaly = False
    anomaly_start = None
    current_anomaly = {'dx': [], 'dy': [], 'dz': []}

    for i, row in data.iterrows():
        time = row['Time (s)']
        dx, dy, dz = row['dx'], row['dy'], row['dz']

        is_anomaly = False

        if not (thresholds['dx'][0] <= dx <= thresholds['dx'][1]):
            is_anomaly = True
        
        if not (thresholds['dy'][0] <= dy <= thresholds['dy'][1]):
            is_anomaly = True
        
        if not (thresholds['dz'][0] <= dz <= thresholds['dz'][1]):
            is_anomaly = True

        if is_anomaly:
            if not in_anomaly:
                anomaly_start = time
                in_anomaly = True
            current_anomaly['dx'].append(dx)
            current_anomaly['dy'].append(dy)
            current_anomaly['dz'].append(dz)
        else:
            if in_anomaly:
                anomalies.append({
                    'start_time': anomaly_start,
                    'end_time': time,
                    'dx_range': (min(current_anomaly['dx']), max(current_anomaly['dx'])),
                    'dy_range': (min(current_anomaly['dy']), max(current_anomaly['dy'])),
                    'dz_range': (min(current_anomaly['dz']), max(current_anomaly['dz'])),
                })
                in_anomaly = False
                current_anomaly = {'dx': [], 'dy': [], 'dz': []}
    
    if in_anomaly:
        anomalies.append({
            'start_time': anomaly_start,
            'end_time': data['Time (s)'].iloc[-1],
            'dx_range': (min(current_anomaly['dx']), max(current_anomaly['dx'])),
            'dy_range': (min(current_anomaly['dy']), max(current_anomaly['dy'])),
            'dz_range': (min(current_anomaly['dz']), max(current_anomaly['dz'])),
        })

    return anomalies

def merge_close_anomalies(anomalies, time_threshold=0.5):
    if not anomalies:
        return []  # Keine Anomalien vorhanden, also gibt es nichts zu mergen

    merged_anomalies = []
    current_anomaly = anomalies[0]

    for next_anomaly in anomalies[1:]:
        if next_anomaly['start_time'] - current_anomaly['end_time'] <= time_threshold:
            current_anomaly['end_time'] = next_anomaly['end_time']
            current_anomaly['dx_range'] = (
                min(current_anomaly['dx_range'][0], next_anomaly['dx_range'][0]),
                max(current_anomaly['dx_range'][1], next_anomaly['dx_range'][1])
            )
            current_anomaly['dy_range'] = (
                min(current_anomaly['dy_range'][0], next_anomaly['dy_range'][0]),
                max(current_anomaly['dy_range'][1], next_anomaly['dy_range'][1])
            )
            current_anomaly['dz_range'] = (
                min(current_anomaly['dz_range'][0], next_anomaly['dz_range'][0]),
                max(current_anomaly['dz_range'][1], next_anomaly['dz_range'][1])
            )
        else:
            merged_anomalies.append(current_anomaly)
            current_anomaly = next_anomaly

    merged_anomalies.append(current_anomaly)
    return merged_anomalies

def print_anomalies(anomalies):
    """Print anomalies in the terminal."""
    if not anomalies:
        print("No anomalies detected.")
        return

    for anomaly in anomalies:
        print(f"Anomaly detected from {anomaly['start_time']:.3f}s to {anomaly['end_time']:.3f}s")
        print(f"  dx: {anomaly['dx_range']}")
        print(f"  dy: {anomaly['dy_range']}")
        print(f"  dz: {anomaly['dz_range']}")
        print()

def main(base_file):
    base_name = os.path.splitext(base_file)[0]
    base_path = os.path.join(os.path.dirname(base_file), base_name)

    csv_file = os.path.join(base_path, 'Gesamt/Gradients.csv')
    output_dir = os.path.join(base_path, 'Gesamt')

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    output_csv = os.path.join(output_dir, 'DetectedAnomalies.csv')
    output_png = os.path.join(output_dir, 'DetectedAnomalies.png')

    data = pd.read_csv(csv_file)

    mean_values = data[['dx', 'dy', 'dz']].mean()

    thresholds = calculate_thresholds(data, mean_values, margin=0.02)

    anomalies = detect_anomalies(data, thresholds)

    merged_anomalies = merge_close_anomalies(anomalies)

    print_anomalies(merged_anomalies)

    if merged_anomalies:
        merged_anomalies_df = pd.DataFrame(merged_anomalies)
        merged_anomalies_df.to_csv(output_csv, index=False)

    plt.figure(figsize=(10, 8))

    for i, axis in enumerate(['dx', 'dy', 'dz'], start=1):
        plt.subplot(3, 1, i)
        plt.plot(data['Time (s)'], data[axis], label=axis, color='red')
        plt.xlabel('Time (s)')
        plt.ylabel(f'Gradient ({axis})')
        plt.grid(True)

        for anomaly in merged_anomalies:
            if (axis == 'dx' and (anomaly['dx_range'][0] != anomaly['dx_range'][1])) or \
               (axis == 'dy' and (anomaly['dy_range'][0] != anomaly['dy_range'][1])) or \
               (axis == 'dz' and (anomaly['dz_range'][0] != anomaly['dz_range'][1])):
                plt.axvspan(anomaly['start_time'], anomaly['end_time'], color='yellow', alpha=0.3)

        plt.legend()

    plt.tight_layout()
    plt.savefig(output_png)

if len(sys.argv) > 1:
    base_file = sys.argv[1]
    print(f"Using the provided file: {base_file}")
else:
    print("No file provided.")
    base_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"

main(base_file)
