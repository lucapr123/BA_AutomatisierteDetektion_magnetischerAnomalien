import sys
import matplotlib
matplotlib.use('Agg')
import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
import time

def save_data_to_csv(data, output_file):
    """Speichert den gegebenen DataFrame in einer CSV-Datei."""
    data.to_csv(output_file, index=False)
    print(f"Data saved to {output_file}")

def save_gradients_to_csv(data, output_file):
    """Berechnet die Gradienten und speichert sie in einer CSV-Datei."""
    gradient_data = pd.DataFrame({
        'Time (s)': data["Time (ms)"] / 1000,
        'dx': data[f'Sensor 1: Achse x'] - data[f'Sensor 2: Achse x'],
        'dy': data[f'Sensor 1: Achse y'] - data[f'Sensor 2: Achse y'],
        'dz': data[f'Sensor 1: Achse z'] - data[f'Sensor 2: Achse z']
    })
    
    gradient_data.to_csv(output_file, index=False)
    print(f"Gradients saved to {output_file}")
    return gradient_data

def integrate_axis(data, axis):
    """Führt die Integration der Achsen über die Zeit durch."""
    integrated_values = np.cumsum(data[axis].values * np.gradient(data['Time (s)'].values))
    return integrated_values

def filter_and_shift_data(data):
    """Verschiebt die Daten um den Mittelwert und filtert sie mit einem gleitenden Durchschnitt."""
    data_copy = data.copy()

    # Verschiebung der Daten
    data_copy['dx'] -= data_copy['dx'].mean()
    data_copy['dy'] -= data_copy['dy'].mean()
    data_copy['dz'] -= data_copy['dz'].mean()

    # Filterung mit einem gleitenden Durchschnitt
    window_size = 5
    data_copy['dx'] = data_copy['dx'].rolling(window=window_size).mean()
    data_copy['dy'] = data_copy['dy'].rolling(window=window_size).mean()
    data_copy['dz'] = data_copy['dz'].rolling(window=window_size).mean()

    data_copy = data_copy.dropna()  # Entfernt NaN-Werte, die durch die Filterung entstehen

    return data_copy

def process_and_save_last_30_seconds(csv_file, axs, fig):
    """Prozessiert die letzten 30 Sekunden der Daten und speichert die Ergebnisse."""
    try:
        data = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # Ordner erstellen, der denselben Namen wie die CSV-Datei hat
    output_folder = f"{csv_file[:-4]}"
    os.makedirs(output_folder, exist_ok=True)

    # Wenn die Datei nicht bereits Gradienten enthält, berechnen wir diese
    if 'dx' not in data.columns or 'dy' not in data.columns or 'dz' not in data.columns:
        print("Berechne Gradienten aus Rohdaten...")
        gradient_output_file = os.path.join(output_folder, "Gradients.csv")
        data = save_gradients_to_csv(data, gradient_output_file)

    # Letzte 30 Sekunden der Daten filtern
    current_time = data["Time (s)"].max()
    min_time = max(0, current_time - 30)
    latest_data = data[data["Time (s)"] >= min_time]

    if latest_data.empty:
        print("Keine neuen Daten im letzten 30-Sekunden-Bereich.")
        return

    # Daten filtern und verschieben
    filtered_data = filter_and_shift_data(latest_data)

    # Integrierte Werte berechnen
    integrated_dx = integrate_axis(filtered_data, 'dx')
    integrated_dy = integrate_axis(filtered_data, 'dy')
    integrated_dz = integrate_axis(filtered_data, 'dz')

    # Berechnung des Künstlichen Totalfelds
    total_field = np.sqrt(integrated_dx**2 + integrated_dy**2 + integrated_dz**2)

    # Speichern der gefilterten Daten als CSV
    filtered_data_csv_file = os.path.join(output_folder, "FilteredGradients.csv")
    save_data_to_csv(filtered_data, filtered_data_csv_file)

    # Speichern der integrierten Daten und des Totalfelds als CSV
    integrated_data = pd.DataFrame({
        'Time (s)': filtered_data['Time (s)'],
        'Integrated dx': integrated_dx,
        'Integrated dy': integrated_dy,
        'Integrated dz': integrated_dz,
        'Total Field': total_field
    })
    integrated_data_csv_file = os.path.join(output_folder, "IntegratedTotalGradients.csv")
    save_data_to_csv(integrated_data, integrated_data_csv_file)

    # Plot der integrierten Werte
    axs[0].clear()
    axs[0].plot(filtered_data['Time (s)'], integrated_dx, label='Integrated DX Axis (Filtered)')
    axs[0].plot(filtered_data['Time (s)'], integrated_dy, label='Integrated DY Axis (Filtered)')
    axs[0].plot(filtered_data['Time (s)'], integrated_dz, label='Integrated DZ Axis (Filtered)')
    axs[0].set_title('Integrated Axis Values Over Last 30 Seconds')
    axs[0].set_xlabel('Time (seconds)')
    axs[0].set_ylabel('Integrated Value')
    axs[0].set_ylim(-0.05, 0.05)  # Set fixed y-axis limits
    axs[0].legend()
    axs[0].grid(True)

    # Plot des Künstlichen Totalfelds
    axs[1].clear()
    axs[1].plot(filtered_data['Time (s)'], total_field, label='Total Field', color='purple')
    axs[1].set_title('Total Field Over Last 30 Seconds')
    axs[1].set_xlabel('Time (seconds)')
    axs[1].set_ylabel('Total Field Value')
    axs[1].set_ylim(0, 0.05)  # Set fixed y-axis limits
    axs[1].legend()
    axs[1].grid(True)

    fig.tight_layout()

    # Save plots
    plot_file = os.path.join(output_folder, "IntegrierteGradients.png")
    fig.savefig(plot_file, bbox_inches='tight')
    print(f"Integrated plot saved as '{plot_file}'")

def run_repeatedly(csv_file, interval=30):
    """Wiederholt die Verarbeitung der letzten 30 Sekunden der CSV-Datei alle 'interval' Sekunden."""
    fig, axs = plt.subplots(2, 1, figsize=(12, 12))

    while True:
        process_and_save_last_30_seconds(csv_file, axs, fig)
        time.sleep(interval)  # Update after every interval

# Überprüfen, ob ein Argument übergeben wurde
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
    print(f"Verwende den übergebenen Dateipfad: {csv_file}")
else:
    print("Kein Dateipfad übergeben.")
    # Optional: Einen Standardpfad setzen oder das Skript beenden
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"

run_repeatedly(csv_file, interval=10)
