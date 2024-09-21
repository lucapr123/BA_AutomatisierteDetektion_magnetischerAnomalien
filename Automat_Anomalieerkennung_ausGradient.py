import os
import sys
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
import time

def calculate_thresholds(data, mean_values, margin=0.025):
    """Berechnet die Schwellenwerte als Mittelwert ± Rand (margin)."""
    return {
        'dx': (mean_values['dx'] - margin, mean_values['dx'] + margin),
        'dy': (mean_values['dy'] - margin, mean_values['dy'] + margin),
        'dz': (mean_values['dz'] - margin, mean_values['dz'] + margin)
    }

def detect_anomalies(data, thresholds):
    """Erkennt Anomalien basierend auf den Schwellenwerten."""
    anomalies = []
    in_anomaly = False
    anomaly_start = None
    current_anomaly = {'dx': [], 'dy': [], 'dz': []}

    # Durchläuft jede Zeile der Daten, um Anomalien zu erkennen
    for i, row in data.iterrows():
        time = row['Time (s)']  # Zeitstempel
        dx, dy, dz = row['dx'], row['dy'], row['dz']  # Gradientenwerte

        is_anomaly = False

        # Überprüft, ob dx außerhalb der Schwellenwerte liegt
        if not (thresholds['dx'][0] <= dx <= thresholds['dx'][1]):
            is_anomaly = True
        
        # Überprüft, ob dy außerhalb der Schwellenwerte liegt
        if not (thresholds['dy'][0] <= dy <= thresholds['dy'][1]):
            is_anomaly = True
        
        # Überprüft, ob dz außerhalb der Schwellenwerte liegt
        if not (thresholds['dz'][0] <= dz <= thresholds['dz'][1]):
            is_anomaly = True

        # Wenn eine Anomalie vorliegt, wird sie markiert
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
    
    # Schließt die letzte Anomalie ab, falls vorhanden
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
    """Fasst nahe beieinanderliegende Anomalien zusammen."""
    if not anomalies:
        return []  # Keine Anomalien zum Zusammenführen

    merged_anomalies = []
    current_anomaly = anomalies[0]

    for next_anomaly in anomalies[1:]:
        # Überprüft, ob die nächste Anomalie innerhalb des Zeitrahmens liegt
        if next_anomaly['start_time'] - current_anomaly['end_time'] <= time_threshold:
            # Zusammenführen der Anomalien, wenn sie zeitlich nahe sind
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

def print_anomalies(anomalies, printed_anomalies):
    """Druckt Anomalien im Terminal aus, wenn sie noch nicht gedruckt wurden."""
    if not anomalies:
        print("No anomalies detected.")
        return

    for anomaly in anomalies:
        anomaly_tuple = (anomaly['start_time'], anomaly['end_time'],
                         tuple(anomaly['dx_range']), tuple(anomaly['dy_range']), tuple(anomaly['dz_range']))
        if anomaly_tuple not in printed_anomalies:
            # Ausgabe der Anomalien
            print(f"Anomaly detected from {anomaly['start_time']:.3f}s to {anomaly['end_time']:.3f}s")
            print(f"  dx: {anomaly['dx_range']}")
            print(f"  dy: {anomaly['dy_range']}")
            print(f"  dz: {anomaly['dz_range']}")
            print()
            printed_anomalies.add(anomaly_tuple)

def save_anomalies_to_csv(anomalies, csv_file):
    """Speichert erkannte Anomalien in einer CSV-Datei, ohne Duplikate."""
    if not anomalies:
        return

    try:
        if os.path.isfile(csv_file):
            existing_anomalies = pd.read_csv(csv_file)  # Bestehende Anomalien laden
        else:
            existing_anomalies = pd.DataFrame()

        df = pd.DataFrame(anomalies)

        # Neue Anomalien anhängen und Duplikate entfernen
        if not existing_anomalies.empty:
            df = pd.concat([existing_anomalies, df]).drop_duplicates().reset_index(drop=True)

        df.to_csv(csv_file, index=False)  # Anomalien in CSV speichern
    except Exception as e:
        print(f"Error saving anomalies to CSV: {e}")

def update_plot(base_file):
    """Aktualisiert den Plot und speichert ihn als PNG."""
    base_name = os.path.splitext(base_file)[0]
    base_path = os.path.join(os.path.dirname(base_file), base_name)

    csv_file = os.path.join(base_path, 'gradients.csv')
    anomalies_csv = "/home/ghost/Magnetometer/Datenaufbereitung/Detektierte_Anomalien.csv"

    printed_anomalies = set()

    fig, axes = plt.subplots(3, 1, figsize=(10, 8))  # Subplots für dx, dy und dz
    axes_dict = {'dx': axes[0], 'dy': axes[1], 'dz': axes[2]}

    while True:
        try:
            data = pd.read_csv(csv_file)
            
            if data.empty:
                print("CSV file is empty.")
                time.sleep(5)
                continue

            # Berechnet Mittelwerte und Schwellenwerte
            mean_values = data[['dx', 'dy', 'dz']].mean()
            thresholds = calculate_thresholds(data, mean_values)

            # Anomalien erkennen und zusammenführen
            anomalies = detect_anomalies(data, thresholds)
            merged_anomalies = merge_close_anomalies(anomalies)

            # Anomalien ausgeben und speichern
            print_anomalies(merged_anomalies, printed_anomalies)
            save_anomalies_to_csv(merged_anomalies, anomalies_csv)

            # Aktualisieren der Plots für jede Achse
            for axis_name, ax in axes_dict.items():
                gradient = data[axis_name]
                gradient_min = gradient.min()
                gradient_max = gradient.max()

                y_axis_min = gradient_min - 0.2
                y_axis_max = gradient_max + 0.2

                # Sicherstellen, dass der Plotbereich nicht zu klein ist
                if y_axis_max - y_axis_min < 0.4:
                    center = (y_axis_max + y_axis_min) / 2
                    y_axis_min = center - 0.2
                    y_axis_max = center + 0.2

                ax.clear()  # Achse leeren
                ax.plot(data['Time (s)'], gradient, label=axis_name, color='red')  # Daten plotten
                ax.set_xlabel('Time (s)')
                ax.set_ylabel(f'Gradient ({axis_name})')
                ax.set_ylim(y_axis_min, y_axis_max)
                ax.grid(True)

                # Markiere Anomalien auf dem Plot
                for anomaly in merged_anomalies:
                    if (axis_name == 'dx' and anomaly['dx_range'][0] != anomaly['dx_range'][1]) or \
                       (axis_name == 'dy' and anomaly['dy_range'][0] != anomaly['dy_range'][1]) or \
                       (axis_name == 'dz' and anomaly['dz_range'][0] != anomaly['dz_range'][1]):
                        ax.axvspan(anomaly['start_time'], anomaly['end_time'], color='yellow', alpha=0.3)

                ax.legend()

            fig.tight_layout()

            # Speichern des Plots als PNG
            plot_file = f"{base_file[:-4]}_anomalies.png"
            fig.savefig(plot_file, bbox_inches='tight')
            print(f"Plot saved as '{plot_file}'")

            time.sleep(5)

        except pd.errors.EmptyDataError:
            print("No columns to parse from file.")
            time.sleep(5)
        except Exception as e:
            print(f"Error while processing: {e}")
            time.sleep(5)

if __name__ == "__main__":
    # Prüft, ob eine Datei als Argument übergeben wurde
    if len(sys.argv) > 1:
        base_file = sys.argv[1]
        print(f"Using the provided file: {base_file}")
    else:
        print("No file provided.")
        csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_2.csv"

    update_plot(base_file)  # Startet den Plot-Update-Prozess
