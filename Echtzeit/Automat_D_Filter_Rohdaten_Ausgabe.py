import matplotlib
matplotlib.use('Agg')
import sys
import csv
import matplotlib.pyplot as plt
import time
import os


# Verwenden des nicht-interaktiven Agg-Backends!!!!!


def read_csv_file(csv_file):
    """Liest eine CSV-Datei und gibt die Daten als Liste von Dictionaries zurück."""
    with open(csv_file, mode='r') as file:
        reader = csv.DictReader(file)
        data = []
        # Jede Zeile der CSV-Datei wird in ein Dictionary konvertiert
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
        return data  # Gibt die gelesenen Daten zurück

def plot_live(csv_file, interval=1, output_file='live_plot.png', initial_range=(0, 3), min_span=0.75):
    """
    Visualisiert Live-Daten aus einer CSV-Datei für jede Achse (x, y, z), zeigt die letzten 30 Sekunden an und speichert
    den Plot als PNG-Datei.
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))  # Erstellt drei Subplots für die Achsen x, y und z
    axes_names = ['x', 'y', 'z']  # Achsenbezeichnungen für die Sensoren
    
    lines = {'sensor1': [], 'sensor2': []}  # Speicher für die Linienobjekte der beiden Sensoren
    
    # Initialisiere die Plots für beide Sensoren und alle Achsen (x, y, z)
    for i, ax in enumerate(axs):
        lines['sensor1'].append(ax.plot([], [], label=f'Sensor 1: Achse {axes_names[i]}')[0])  # Linie für Sensor 1
        lines['sensor2'].append(ax.plot([], [], label=f'Sensor 2: Achse {axes_names[i]}')[0])  # Linie für Sensor 2
        ax.set_xlim(0, 30)  # Zeitachse auf die letzten 30 Sekunden setzen
        ax.set_ylim(initial_range)  # Y-Achse initial auf den angegebenen Bereich setzen
        ax.set_xlabel('Time (s)')  # Beschriftung der X-Achse
        ax.set_ylabel(f'Achse {axes_names[i]}')  # Beschriftung der Y-Achse
        ax.legend()  # Legende anzeigen
        ax.grid(True)  # Gitternetz aktivieren
    
    plt.tight_layout()  # Layout anpassen

    while True:
        try:
            data = read_csv_file(csv_file)  # Liest die CSV-Datei
            
            # Konvertiert die Zeit von Millisekunden in Sekunden
            times = [row['Time (ms)'] / 1000 for row in data]
            current_time = max(times)  # Aktuelle Zeit
            min_time = max(0, current_time - 30)  # Zeitspanne der letzten 30 Sekunden
            
            # Filtert die Daten, um nur die letzten 30 Sekunden anzuzeigen
            filtered_data = [row for row in data if row['Time (ms)'] / 1000 >= min_time]
            
            spans = []  # Speichert die Spannen (Min-Max-Differenz) für jede Achse
            for i, axis in enumerate(axes_names):
                y_min = min(row[f'Sensor 1: Achse {axis}'] for row in filtered_data)  # Minimum der Achse
                y_max = max(row[f'Sensor 1: Achse {axis}'] for row in filtered_data)  # Maximum der Achse
                spans.append(y_max - y_min)  # Spannweite der Achse
            
            max_span = max(max(spans), min_span)  # Maximale Spannweite sicherstellen
            margin = 0.05 * max_span  # Einen kleinen Rand hinzufügen
            
            for i, axis in enumerate(axes_names):
                y_min = min(row[f'Sensor 1: Achse {axis}'] for row in filtered_data)  # Minimum für diese Achse
                y_max = max(row[f'Sensor 1: Achse {axis}'] for row in filtered_data)  # Maximum für diese Achse
                
                # Zentrum und neue y-Achsenlimits berechnen
                center = (y_max + y_min) / 2
                new_y_min = center - max_span / 2 - margin
                new_y_max = center + max_span / 2 + margin
                
                # Aktualisiere die Daten für die beiden Sensoren
                lines['sensor1'][i].set_data([row['Time (ms)'] / 1000 for row in filtered_data], 
                                             [row[f'Sensor 1: Achse {axis}'] for row in filtered_data])
                lines['sensor2'][i].set_data([row['Time (ms)'] / 1000 for row in filtered_data], 
                                             [row[f'Sensor 2: Achse {axis}'] for row in filtered_data])
                
                # Setze die x- und y-Achsenbegrenzungen
                axs[i].set_xlim(min_time, current_time)
                axs[i].set_ylim(new_y_min, new_y_max)
                axs[i].relim()
                axs[i].autoscale_view()
            
            plt.draw()  # Zeichnet den Plot neu
            plt.pause(interval)  # Wartet für das angegebene Intervall
            
            # Sicherstellen, dass das Verzeichnis existiert
            output_dir = os.path.dirname(output_file)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)  # Verzeichnis erstellen, falls es nicht existiert

            # Speichern des Plots als PNG-Datei
            plt.savefig(output_file, bbox_inches='tight')
            print(f"Saved plot to {output_file}")  # Ausgabe, dass der Plot gespeichert wurde
            
        except Exception as e:
            print(f"Error updating plot: {e}")  # Fehlerbehandlung bei der Plot-Aktualisierung
            continue

if __name__ == "__main__":
    # Überprüft, ob eine CSV-Datei als Argument übergeben wurde
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
        print(f"Verwende den übergebenen Dateipfad: {csv_file}")  # Ausgabe des verwendeten Dateipfads
    else:
        print("Kein Dateipfad übergeben.")
        csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"  # Standardpfad, falls keiner übergeben wurde

    time.sleep(1)  # Kurze Pause, bevor die Live-Visualisierung beginnt
    print("Starte die Live-Visualisierung...")
    # Starte die Live-Visualisierung und speichere den Plot in einem neuen Verzeichnis
    plot_live(csv_file, interval=1, output_file= f"{csv_file[:-4]}/Rohdaten.png")
