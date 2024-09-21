import sys
import matplotlib
matplotlib.use('Agg')  # Verwenden des nicht-interaktiven Backends für das Rendern von Grafiken ohne GUI
import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
import time

def save_data_to_csv(data, output_file):
    """Speichert den gegebenen DataFrame in eine CSV-Datei."""
    data.to_csv(output_file, index=False)  # Speichert die Daten ohne Index in die CSV-Datei
    print(f"Data saved to {output_file}")

def save_gradients_to_csv(data, output_file):
    """Berechnet die Gradienten zwischen den Sensorachsen und speichert sie in einer CSV-Datei."""
    # Berechnet die Differenzen der Achsenwerte zwischen Sensor 1 und Sensor 2
    gradient_data = pd.DataFrame({
        'Time (s)': data["Time (ms)"] / 1000,  # Konvertiert die Zeit von Millisekunden in Sekunden
        'dx': data[f'Sensor 1: Achse x'] - data[f'Sensor 2: Achse x'],
        'dy': data[f'Sensor 1: Achse y'] - data[f'Sensor 2: Achse y'],
        'dz': data[f'Sensor 1: Achse z'] - data[f'Sensor 2: Achse z']
    })
    
    # Speichert die Gradienten in eine CSV-Datei
    gradient_data.to_csv(output_file, index=False)
    print(f"Gradients saved to {output_file}")
    return gradient_data

def integrate_axis(data, axis):
    """Führt die Integration der Werte einer Achse (dx, dy, dz) über die Zeit durch."""
    integrated_values = np.cumsum(data[axis].values * np.gradient(data['Time (s)'].values))  # Numerische Integration der Achsenwerte
    return integrated_values

def filter_and_shift_data(data):
    """Verschiebt die Achsenwerte um den Mittelwert und filtert die Daten mit einem gleitenden Durchschnitt."""
    data_copy = data.copy()  # Erstellt eine Kopie der Daten, um die Originaldaten nicht zu verändern

    # Verschiebt die Achsenwerte um ihren jeweiligen Mittelwert
    data_copy['dx'] -= data_copy['dx'].mean()
    data_copy['dy'] -= data_copy['dy'].mean()
    data_copy['dz'] -= data_copy['dz'].mean()

    # Wendet einen gleitenden Durchschnitt mit einem Fenster von 5 Werten an
    window_size = 5
    data_copy['dx'] = data_copy['dx'].rolling(window=window_size).mean()
    data_copy['dy'] = data_copy['dy'].rolling(window=window_size).mean()
    data_copy['dz'] = data_copy['dz'].rolling(window=window_size).mean()

    data_copy = data_copy.dropna()  # Entfernt Zeilen mit NaN-Werten, die durch den gleitenden Durchschnitt entstehen

    return data_copy

def process_and_save_last_30_seconds(csv_file, axs, fig):
    """Verarbeitet die letzten 30 Sekunden der Daten und speichert die Ergebnisse."""
    try:
        # Liest die CSV-Datei in einen DataFrame
        data = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # Erstellt einen Ordner, in dem die Ergebnisse gespeichert werden
    output_folder = f"{csv_file[:-4]}"
    os.makedirs(output_folder, exist_ok=True)

    # Wenn die Datei noch keine Gradienten enthält, werden diese berechnet
    if 'dx' not in data.columns or 'dy' not in data.columns or 'dz' not in data.columns:
        print("Berechne Gradienten aus Rohdaten...")
        gradient_output_file = os.path.join(output_folder, "Gradients.csv")
        data = save_gradients_to_csv(data, gradient_output_file)

    # Filtert die letzten 30 Sekunden der Daten
    current_time = data["Time (s)"].max()  # Ermittelt die aktuelle Zeit
    min_time = max(0, current_time - 30)  # Bestimmt die Mindestzeit (30 Sekunden vor der aktuellen Zeit)
    latest_data = data[data["Time (s)"] >= min_time]  # Filtert die Daten der letzten 30 Sekunden

    if latest_data.empty:
        print("Keine neuen Daten im letzten 30-Sekunden-Bereich.")
        return

    # Filtert und verschiebt die Daten um den Mittelwert
    filtered_data = filter_and_shift_data(latest_data)

    # Integriert die Achsenwerte (dx, dy, dz) über die Zeit
    integrated_dx = integrate_axis(filtered_data, 'dx')
    integrated_dy = integrate_axis(filtered_data, 'dy')
    integrated_dz = integrate_axis(filtered_data, 'dz')

    # Berechnet das künstliche Totalfeld
    total_field = np.sqrt(integrated_dx**2 + integrated_dy**2 + integrated_dz**2)

    # Speichert die gefilterten Daten in einer CSV-Datei
    filtered_data_csv_file = os.path.join(output_folder, "FilteredGradients.csv")
    save_data_to_csv(filtered_data, filtered_data_csv_file)

    # Speichert die integrierten Daten und das Totalfeld in einer CSV-Datei
    integrated_data = pd.DataFrame({
        'Time (s)': filtered_data['Time (s)'],
        'Integrated dx': integrated_dx,
        'Integrated dy': integrated_dy,
        'Integrated dz': integrated_dz,
        'Total Field': total_field
    })
    integrated_data_csv_file = os.path.join(output_folder, "IntegratedTotalGradients.csv")
    save_data_to_csv(integrated_data, integrated_data_csv_file)

    # Plotten der integrierten Werte (dx, dy, dz)
    axs[0].clear()
    axs[0].plot(filtered_data['Time (s)'], integrated_dx, label='Integrated DX Axis (Filtered)')
    axs[0].plot(filtered_data['Time (s)'], integrated_dy, label='Integrated DY Axis (Filtered)')
    axs[0].plot(filtered_data['Time (s)'], integrated_dz, label='Integrated DZ Axis (Filtered)')
    axs[0].set_title('Integrated Axis Values Over Last 30 Seconds')
    axs[0].set_xlabel('Time (seconds)')
    axs[0].set_ylabel('Integrated Value')
    axs[0].set_ylim(-0.05, 0.05)  # Feste y-Achsenbegrenzungen
    axs[0].legend()
    axs[0].grid(True)

    # Plotten des künstlichen Totalfelds
    axs[1].clear()
    axs[1].plot(filtered_data['Time (s)'], total_field, label='Total Field', color='purple')
    axs[1].set_title('Total Field Over Last 30 Seconds')
    axs[1].set_xlabel('Time (seconds)')
    axs[1].set_ylabel('Total Field Value')
    axs[1].set_ylim(0, 0.05)  # Feste y-Achsenbegrenzungen
    axs[1].legend()
    axs[1].grid(True)

    fig.tight_layout()

    # Speichert den Plot als PNG-Datei
    plot_file = os.path.join(output_folder, "IntegrierteGradients.png")
    fig.savefig(plot_file, bbox_inches='tight')
    print(f"Integrated plot saved as '{plot_file}'")

def run_repeatedly(csv_file, interval=30):
    """Wiederholt die Verarbeitung der letzten 30 Sekunden der CSV-Datei alle 'interval' Sekunden."""
    fig, axs = plt.subplots(2, 1, figsize=(12, 12))  # Erstellt die Subplots für die Daten

    while True:
        process_and_save_last_30_seconds(csv_file, axs, fig)  # Verarbeitet die Daten und aktualisiert die Plots
        time.sleep(interval)  # Wartet das angegebene Intervall, bevor die nächste Verarbeitung beginnt

# Überprüfen, ob ein Dateipfad als Argument übergeben wurde
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
    print(f"Verwende den übergebenen Dateipfad: {csv_file}")
else:
    print("Kein Dateipfad übergeben.")
    # Standard-Dateipfad festlegen, wenn kein Argument übergeben wurde
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"

run_repeatedly(csv_file, interval=10)  # Wiederholt die Verarbeitung alle 10 Sekunden
