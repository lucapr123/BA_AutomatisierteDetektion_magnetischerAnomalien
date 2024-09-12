import os
import subprocess
import time

# Pfad zur Datei mit den Prozess-IDs (dieser wird nicht mehr verwendet, kann entfernt werden)
file_path = "/home/ghost/Magnetometer/Datenaufbereitung/XLaufende_Prozesse.txt"

print("Beenden der Auswertung.....")

# Beenden aller Prozesse, die Python-Skripte im Verzeichnis '/home/ghost/Magnetometer/' ausführen
try:
    # Beende alle Prozesse, die mit 'python3 /home/ghost/Magnetometer' gestartet wurden
    subprocess.call(['pkill', '-f', 'python3 /home/ghost/Magnetometer'])
    print("Alle relevanten Python-Prozesse wurden beendet.")
except Exception as e:
    print(f"Fehler beim Beenden der Prozesse: {e}")

time.sleep(2)

# Führe das Gesamt_Rohdaten.py Skript aus
try:
    subprocess.Popen(["python3", "/home/ghost/Magnetometer/Datenaufbereitung/Gesamt_Plots/Gesamt_launch.py"])
    print("Erzeugen von Plots der gesamten Messung.......")
except Exception as e:
    print(f"Failed to start Gesamt_Rohdaten.py: {e}")
