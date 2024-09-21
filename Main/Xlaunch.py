import subprocess
import time
import os

def run_script(script_name, dateipfad):
    """Runs a Python script after setting up the ROS 2 environment and passes the file path as an argument."""
    
    # Befehl vorbereiten
    full_command = (
        f'bash -c "source /home/ghost/Magnetometer/ros2_workspace/install/setup.bash && '
        f'python3 {script_name} {dateipfad} &"'  # Das '&' startet den Prozess im Hintergrund
    )

    # Befehl zur Debugging-Zwecken ausgeben
    print(f"Executing command: {full_command}")

    # Starte das Skript als separaten Prozess im Hintergrund
    try:
        subprocess.Popen(full_command, shell=True)

    except Exception as e:
        print(f"Error during execution of script {script_name}: {e}")

# Dateipfad, der an die Skripte übergeben wird
dateipfad = "/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/Messung_aktuell.csv"
print(f"Dateipfad: {dateipfad}")

# Erstellen des Verzeichnisses für die Ergebnisse
Messung_Folder = f"{dateipfad[:-4]}/"
os.makedirs(Messung_Folder, exist_ok=True)
Gesamt = f"{Messung_Folder}/Gesamt"
os.makedirs(Gesamt, exist_ok=True)

# Ändere den Besitzer aller Dateien in Messung_Folder auf ghost:ghost
subprocess.call(['chown', '-R', 'ghost:ghost', Messung_Folder])

if __name__ == "__main__":
    
    print(f"Using CSV file: {dateipfad}")

    # Liste der Skripte, die ausgeführt werden sollen
    scripts = [
        "/home/ghost/Magnetometer/Datenaufbereitung/Visualisierung_Zeitlich/D_Filter_Datenerfassung_Stabil_Serial_to_csv_Validiert.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Visualisierung_Zeitlich/Automat_D_Filter_Rohdaten_Ausgabe.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Visualisierung_Zeitlich/Automat_Gradient_aus_Rohdaten.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Visualisierung_Zeitlich/Automat_TotalGradient.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Visualisierung_Zeitlich/Automat_Magnet-Totalfeld_aus_Gradient_aus_Rohdaten.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Visualisierung_Zeitlich/Automat_Vektor_Gradienten_3D.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Visualisierung_Zeitlich/Automat_Anomalieerkennung_ausGradient.py",
        "/home/ghost/Magnetometer/new_Flask/app_neu.py",
        "/home/ghost/Magnetometer/ros2_workspace/src/publishing_terminal/publishing_terminal/publishing_terminal_node.py",
        "/home/ghost/Magnetometer/ros2_workspace/src/publishing_plots/publishing_plots/publishing_plots_node.py"
    ]

    # Starte alle Skripte
    for script in scripts:
        print(f"Starte Skript: {script}")
        run_script(script, dateipfad)
        
        # Kurze Pause zwischen den Skriptstarts, um Ressourcenbelastung zu vermeiden
        time.sleep(2)

    print("All scripts started.")
