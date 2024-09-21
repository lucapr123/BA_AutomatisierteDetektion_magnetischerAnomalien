import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 3D-Plotting-Toolkit
import numpy as np
import sys
import matplotlib

# Verwende das nicht-interaktive 'Agg'-Backend, das für das Rendern von Grafiken ohne GUI verwendet wird
matplotlib.use('Agg')

def plot_3d_magnetic_field(csv_file):
    """Erstellt eine 3D-Visualisierung des magnetischen Feldes aus einer CSV-Datei, wobei 'dy' die Farbskala repräsentiert."""
    
    print("Plotting 3D magnetic field visualization with color representing 'dy'...")
    
    # Erstellen einer Figur für den 3D-Plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Setzen der Achsenbeschriftungen und des Titels
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Integrated dx")
    ax.set_zlabel("Integrated dz")
    ax.set_title("3D Visualization of Magnetic Field with Colored 'dy'")

    # Versuch, die Daten aus der CSV-Datei zu lesen
    try:
        data = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")  # Fehlermeldung, falls die Datei nicht gelesen werden kann
        return

    if data.empty:
        print("Keine Daten zum Plotten.")  # Wenn die CSV-Datei leer ist, wird dies ausgegeben
        return

    # Extrahieren der Daten aus dem DataFrame für den Plot
    dx = data['Integrated dx']
    dz = data['Integrated dz']
    dy = data['Integrated dy']  # 'dy'-Werte, die für die Farbgebung verwendet werden
    time_values = data["Time (s)"]

    # Erstellen eines 3D-Streudiagramms, wobei 'dy' als Farbwert verwendet wird
    scatter = ax.scatter(time_values, dx, dz, c=dy, cmap='viridis')

    # Hinzufügen einer Farbskala (Colorbar) zum Plot, um den Bereich von 'dy' darzustellen
    cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
    cbar.set_label('Integrated dy')  # Beschriftung der Farbskala

    # Anpassen der Achsenlimits basierend auf den Daten
    ax.set_xlim(time_values.min(), time_values.max())  # Zeitachse
    ax.set_ylim(dx.min() - 0.1, dx.max() + 0.1)  # Achse für dx
    ax.set_zlim(dz.min() - 0.1, dz.max() + 0.1)  # Achse für dz

    # Speichern des Plots als PNG-Datei
    output_file = csv_file.replace(".csv", "_3Dplot_colored_dy.png")
    plt.savefig(output_file, bbox_inches='tight')
    print(f"Plot saved to {output_file}")  # Ausgabe, wenn der Plot erfolgreich gespeichert wurde

# Überprüfen, ob ein Dateipfad als Argument übergeben wurde
if len(sys.argv) > 1:
    # Verwenden des übergebenen Dateipfads und Ersetzen der .csv-Endung
    csv_file = f"{sys.argv[1][:-4]}/IntegratedTotalGradients.csv"
    print(f"Verwende den übergebenen Dateipfad: {csv_file}")
else:
    # Falls kein Dateipfad übergeben wurde, wird ein Standardpfad verwendet
    print("Kein Dateipfad übergeben.")
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_3/IntegratedTotalGradients.csv"

# Aufruf der Funktion zur Erstellung des 3D-Plots
plot_3d_magnetic_field(csv_file)
