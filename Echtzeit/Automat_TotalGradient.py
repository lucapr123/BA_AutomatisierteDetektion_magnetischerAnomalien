import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Verwenden des nicht-interaktiven Agg-Backends für das Rendern von Grafiken ohne GUI
import matplotlib.pyplot as plt
import time
import numpy as np
import os
import sys

def calculate_total_field(data):
    """Berechnet das Totalfeld (die Magnitude) aus den Gradientenkomponenten dx, dy und dz."""
    total_field = np.sqrt(data['dx']**2 + data['dy']**2 + data['dz']**2)  # Totalfeld berechnet als Quadratwurzel der Summe der Quadrate
    return total_field

def plot_total_field(csv_file, output_file='total_field_plot.png', initial_range=(0.1, 0.3), min_span=0.1):
    """
    Plottet das Totalfeld aus der CSV-Datei, zeigt nur die letzten 30 Sekunden an und speichert den Plot als PNG.
    """
    
    fig, ax = plt.subplots(figsize=(10, 6))  # Erstellen einer neuen Figur und einer Achse für den Plot
    
    while True:  # Dauerschleife zum kontinuierlichen Aktualisieren des Plots
        try:
            data = pd.read_csv(csv_file)  # Einlesen der CSV-Datei
            
            # Filtert die Daten, um nur die letzten 30 Sekunden anzuzeigen
            if 'Time (s)' in data.columns:
                time_column = 'Time (s)'  # Verwendet die Zeit in Sekunden
            else:
                print("Zeitspalte in der CSV-Datei nicht gefunden.")
                return

            current_time = data[time_column].max()  # Aktuelle Zeit ermitteln
            min_time = max(0, current_time - 30)  # Mindestzeit für die letzten 30 Sekunden
            filtered_data = data[data[time_column] >= min_time]  # Filtert die Daten basierend auf der Mindestzeit
            
            # Berechnet das Totalfeld
            total_field = calculate_total_field(filtered_data)
            
            # Dynamische Berechnung der y-Achsen-Grenzen
            y_min, y_max = total_field.min(), total_field.max()  # Minimum und Maximum des Totalfelds
            span = max(y_max - y_min, min_span)  # Stellt sicher, dass die Spannweite mindestens min_span beträgt
            margin = 0.05 * span  # Fügt einen kleinen Rand zu den y-Grenzen hinzu

            center = (y_max + y_min) / 2  # Zentrum der y-Achse
            new_y_min = center - span / 2 - margin
            new_y_max = center + span / 2 + margin

            ax.clear()  # Löscht den Plot, bevor er aktualisiert wird
            
            # Aktualisieren des Plots mit den gefilterten Daten
            ax.plot(filtered_data[time_column], total_field, label='Total Field', color='b')
            ax.set_xlim(min_time, current_time)  # Setzt die x-Achse auf die letzten 30 Sekunden
            ax.set_ylim(0, 0.4)  # Setzt feste Grenzen für die y-Achse
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Total Field')
            ax.legend()  # Fügt eine Legende hinzu
            ax.grid(True)  # Aktiviert das Raster für bessere Sichtbarkeit
            
            # Speichert das Diagramm als PNG-Datei
            plt.savefig(output_file, bbox_inches='tight')
            print(f"Saved plot to {output_file}")  # Ausgabe, wenn der Plot gespeichert wurde

            time.sleep(1)  # Wartet eine Sekunde, bevor der Plot erneut aktualisiert wird
            
        except Exception as e:
            print(f"Fehler beim Aktualisieren des Plots: {e}")
            continue  # Bei einem Fehler wird die Schleife fortgesetzt

# Überprüft, ob ein Dateipfad als Argument übergeben wurde
if len(sys.argv) > 1:
    csv_file = f"{sys.argv[1][:-4]}/gradients.csv"  # Verwendet den übergebenen Dateipfad und ersetzt ihn mit dem Pfad zur 'gradients.csv'-Datei
    print(f"Verwende den übergebenen Dateipfad: {csv_file}")
else:
    print("Kein Dateipfad übergeben.")  # Ausgabe, wenn kein Dateipfad übergeben wurde
    # Optional: Standardpfad setzen oder das Skript beenden
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3.csv"  # Standardpfad zur CSV-Datei

# Startet die Funktion zum Plotten des Totalfelds
plot_total_field(csv_file, output_file= f"{csv_file[:-13]}TotalGradient.png", initial_range=(0, 1), min_span=1)
