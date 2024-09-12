import sys
import time
import subprocess
from datetime import datetime, timedelta
import shutil
import os

def run_script(script_name, dateipfad):
    """Runs a Python script after setting up the ROS 2 environment and passes the file path as an argument."""
    full_command = (
        f'bash -c "source /home/ghost/Magnetometer/ros2_workspace/install/setup.bash && '
        f'python3 {script_name} {dateipfad}"'  # Entferne das '&' um das Skript synchron laufen zu lassen
    )
    
    # Log the full command for debugging purposes
    print(f"Executing command: {full_command}")
    
    try:
        # Starte das Skript im Vordergrund und warte, bis es abgeschlossen ist
        process = subprocess.Popen(full_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        # Print stdout and stderr in real time
        while True:
            output = process.stdout.readline()
            if output == b'' and process.poll() is not None:
                break
            if output:
                print(output.decode().strip())
        
        # Capture stderr
        stderr_output = process.stderr.read().decode().strip()
        if stderr_output:
            print(f"Error output:\n{stderr_output}")
    
        return process
    except Exception as e:
        print(f"Error during execution of script {script_name}: {e}")

# File path to be passed to the scripts
dateipfad = "/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/Messung_aktuell.csv"

if __name__ == "__main__":
    
    print(f"Using CSV file: {dateipfad}")

    # List of scripts to be executed
    scripts = [
        "/home/ghost/Magnetometer/Datenaufbereitung/Gesamt_Plots/Gesamt_Gradienten.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Gesamt_Plots/Gesamt_Magnet-Totalfeld.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Gesamt_Plots/Gesamt_Rohdaten.py",
        "/home/ghost/Magnetometer/Datenaufbereitung/Gesamt_Plots/Gesamt_Anomalieerkennung_ausGradient.py",
        "/home/ghost/Magnetometer/ros2_workspace/src/publishing_plots/publishing_plots/publishing_plots_gesamt.py"
    ]

    # Execute scripts sequentially, passing the file path
    for script in scripts:
        print(f"Starten des Skripts: {script}")
        run_script(script, dateipfad)
            
    print("Plots created.")
    time.sleep(2)
    
    # Archivierung am Ende
    # Archivierungsordner erstellen mit aktuellem Datum und Uhrzeit
    # Aktuelle Zeit und 6 Stunden hinzufügen
    time_with_offset = datetime.now() + timedelta(hours=6)
    # Formatieren der neuen Zeit
    archiv_folder_name = time_with_offset.strftime("Messung_%Y-%m-%d_%H-%M-%S")
    archiv_folder_path = f"/home/ghost/Magnetometer/Datenerfassung/Messungen_alt/{archiv_folder_name}"
    os.makedirs(archiv_folder_path, exist_ok=True)
    print(f"Archivordner erstellt: {archiv_folder_path}")
    # Ändere den Besitzer des Archivierungsordners auf ghost:ghost
    subprocess.call(['chown', '-R', 'ghost:ghost', archiv_folder_path])

    try:
        shutil.move("/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/", archiv_folder_path)
        print(f"Archiviert: {archiv_folder_path}")
    except Exception as e:
        print(f"Fehler bei der Archivierung: {e}")

    # Leeren des Ordners für die nächste Messung
    os.makedirs("/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/", exist_ok=True)
    print("Messung_aktuell für die nächste Messung geleert.")

    time.sleep(2)


    # Beenden aller Prozesse, die Python-Skripte im Verzeichnis '/home/ghost/Magnetometer/' ausführen
    try:
        # Beende alle Prozesse, die mit 'python3 /home/ghost/Magnetometer' gestartet wurden
        subprocess.call(['pkill', '-f', 'python3 /home/ghost/Magnetometer/ros2_workspace/src/publishing_plots/'])
        print("Alle relevanten Python-Prozesse wurden beendet.")
    except Exception as e:
        print(f"Fehler beim Beenden der Prozesse: {e}")

    print("Automatically exiting the application")
    time.sleep(2)

    sys.exit()
