import sys
import matplotlib
matplotlib.use('Agg')  # Verwenden des nicht-interaktiven Agg-Backends für das Rendern von Grafiken
import pandas as pd
import matplotlib.pyplot as plt
import time

def calculate_gradients(data, time_column):
    """Berechnet die Gradienten zwischen den Werten von Sensor 1 und Sensor 2 und gibt das modifizierte DataFrame zurück."""
    data['dx'] = data['Sensor 1: Achse x'] - data['Sensor 2: Achse x']
    data['dy'] = data['Sensor 1: Achse y'] - data['Sensor 2: Achse y']
    data['dz'] = data['Sensor 1: Achse z'] - data['Sensor 2: Achse z']
    return data

def save_gradients_to_csv(data, output_file):
    """Speichert die berechneten Gradienten in eine CSV-Datei."""
    gradient_data = data[['Time (s)', 'dx', 'dy', 'dz']]  # Nur die Zeit- und Gradientenspalten speichern
    gradient_data.to_csv(output_file, index=False)  # Speichert das DataFrame ohne Index in eine CSV-Datei
    print(f"Gradients saved to {output_file}")

def save_plot(data, axs, time_column, output_file, initial_range=(-0.4, 0.4)):
    """Aktualisiert das Plot mit den neuen Daten und speichert es als PNG-Datei."""
    axes_names = ['x', 'y', 'z']  # Achsenbezeichnungen
    
    spans = []
    for axis in axes_names:
        gradient = data[f'd{axis}']  # Extrahiert die Gradienten für jede Achse
        spans.append(gradient.max() - gradient.min())  # Berechnet die Spannweite für jede Achse

    max_span = max(spans)  # Bestimmt die größte Spannweite
    y_axis_margin = 0.05 * max_span  # Hinzufügen einer kleinen Marge für die y-Achse

    # Aktualisiere die drei Subplots für jede Achse
    for i, ax in enumerate(axs):
        gradient = data[f'd{axes_names[i]}']  # Gradienten für die entsprechende Achse
        gradient_min = gradient.min()
        gradient_max = gradient.max()

        # Setze die y-Limits, um eine konsistente Ansicht zu gewährleisten
        if gradient_min < initial_range[0] or gradient_max > initial_range[1]:
            center = (gradient_max + gradient_min) / 2
            y_axis_min = center - max_span / 2 - y_axis_margin
            y_axis_max = center + max_span / 2 + y_axis_margin
        else:
            y_axis_min, y_axis_max = initial_range

        ax.clear()  # Löscht den aktuellen Plot
        ax.plot(data[time_column] / 1000, gradient, label=f'd{axes_names[i]}', color='r')  # Zeichnet den neuen Plot
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Gradient ({axes_names[i]})")
        ax.set_ylim(y_axis_min, y_axis_max)  # Setzt die Grenzen für die y-Achse
        ax.legend()  # Legende hinzufügen
        ax.grid(True)  # Gitternetz aktivieren
    
    plt.savefig(output_file, bbox_inches='tight')  # Speichert den Plot als PNG-Datei
    print(f"Plot saved to {output_file}")

def process_last_30_seconds(csv_file):
    """Verarbeitet die letzten 30 Sekunden der Daten aus der CSV-Datei."""
    try:
        data = pd.read_csv(csv_file)  # Liest die CSV-Datei in ein DataFrame
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return None, None

    # Überprüfen, welche Zeiteinheit verwendet wird (ms oder s)
    if 'Time (ms)' in data.columns:
        time_column = 'Time (ms)'
        data['Time (s)'] = data['Time (ms)'] / 1000  # Konvertiert ms in Sekunden
    elif 'Time (s)' in data.columns:
        time_column = 'Time (s)'
    else:
        print("Time column not found.")
        return None, None

    current_time = data[time_column].max()  # Bestimmt die aktuelle Zeit
    min_time = max(0, current_time - 30000)  # Extrahiert die Daten der letzten 30 Sekunden

    latest_data = data[data[time_column] >= min_time]  # Filtert die letzten 30 Sekunden der Daten

    if latest_data.empty:
        print("No new data in the last 30 seconds.")
        return None, None

    # Berechnet die Gradienten für die letzten 30 Sekunden
    latest_data = calculate_gradients(latest_data, time_column)

    # Speichert die Gradienten in einer CSV-Datei
    output_csv_file = f"{csv_file[:-4]}/gradients.csv"
    save_gradients_to_csv(latest_data, output_csv_file)
    
    return latest_data, 'Time (s)'

def run_repeatedly(csv_file, interval=2):
    """Verarbeitet und speichert die Daten wiederholt alle 'interval' Sekunden."""
    
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))  # Erstellt das Diagramm mit drei Subplots
    
    while True:
        latest_data, time_column = process_last_30_seconds(csv_file)  # Verarbeitet die letzten 30 Sekunden der Daten
        
        if latest_data is not None:
            output_file = f"{csv_file[:-4]}/gradient_plot.png"  # Bestimmt den Speicherort für den Plot
            save_plot(latest_data, axs, time_column, output_file)  # Speichert den aktualisierten Plot
        
        time.sleep(interval)  # Wartezeit vor der nächsten Aktualisierung

# Überprüft, ob ein Dateipfad als Argument übergeben wurde
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
    print(f"Using provided file path: {csv_file}")
else:
    print("No file path provided.")
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"  # Standardpfad, wenn kein Argument übergeben wurde

run_repeatedly(csv_file, interval=1)  # Führt die Verarbeitung wiederholt mit einem Intervall von 1 Sekunde aus
