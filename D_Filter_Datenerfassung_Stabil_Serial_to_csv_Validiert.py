import serial
import csv
import re
import time
import sys
import os

# Konfiguration der seriellen Verbindung
try:
    ser = serial.Serial('/dev/ttyACM1', 115200, timeout=30)  # Erhöhen Sie das Timeout
    time.sleep(2)  # Warte, um sicherzustellen, dass die Verbindung stabil ist
except serial.SerialException as e:
    print(f"Fehler beim Öffnen des seriellen Ports: {e}")
    exit(1)

# Überprüfen, ob ein Argument übergeben wurde
if len(sys.argv) > 1:
    csv_file = sys.argv[1]
    print(f"Verwende den übergebenen Dateipfad: {csv_file}")
else:
    print("Kein Dateipfad übergeben.")
    csv_file = "/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/Messung_aktuell.csv"

last_row = None  # To store the last validated row

def validate_row(data):
    """Validate the extracted data to ensure it's logical and within expected ranges."""
    global last_row
    
    try:
        # Convert each sensor value to float and check if they are within a logical range
        time_value = int(data[0])
        sensor_values = [float(value) for value in data[1:]]
        
        # Example validation checks for value ranges
        if not all(-10.0 <= value <= 10.0 for value in sensor_values):
            print(f"Validation failed for data (out of range): {data}")
            return False
        
        # Additional checks for the difference with the last row
        if last_row is not None:
            last_time_value = int(last_row[0])
            last_sensor_values = [float(value) for value in last_row[1:]]
            
            # Check if the time difference is within 5000 ms
            if abs(time_value - last_time_value) > 5000:
                print(f"Validation failed for data (time difference too large): {data}")
                return False
            
            # Check if the sensor value differences are within 0.5
            if not all(abs(current - last) <= 0.5 for current, last in zip(sensor_values, last_sensor_values)):
                print(f"Validation failed for data (sensor value difference too large): {data}")
                return False
        
        # If all checks pass, update last_row and return True
        last_row = data
        return True
        
    except ValueError:
        print(f"Error converting data to numeric types: {data}")
        return False

try:
    print("Attempting to open CSV file:")
    with open(csv_file, mode='w', newline='') as file:
        print("CSV file opened:")
        writer = csv.writer(file)
        writer.writerow(['Time (ms)', 'Sensor 1: Achse x', 'Sensor 1: Achse y', 'Sensor 1: Achse z', 
                         'Sensor 2: Achse x', 'Sensor 2: Achse y', 'Sensor 2: Achse z'])  # Write header
        print("CSV file header written.")

        line_count = 0  # Counter

        while True:
            if ser.in_waiting > 0:
                print(f"{ser.in_waiting} bytes available in buffer")
                line = ser.readline().decode('utf-8', errors='replace').strip()
                print(f"Line read: {line}")

                line_count += 1  # Increment the counter
                
                if line_count <= 10:
                    continue  # Skip writing the first 10 lines
                
                data_match = re.search(r'Time \(ms\):\s*(\d+),\s*'
                                       r'Raw Sensor 1:\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)\s*\|\s*'
                                       r'Filtered Sensor 1:\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)\s*\|\s*'
                                       r'Raw Sensor 2:\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)\s*\|\s*'
                                       r'Filtered Sensor 2:\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)', line)
                if data_match:
                    time_value = data_match.group(1)
                    sensor1_filtered_x = data_match.group(5)
                    sensor1_filtered_y = data_match.group(6)
                    sensor1_filtered_z = data_match.group(7)
                    sensor2_filtered_x = data_match.group(11)
                    sensor2_filtered_y = data_match.group(12)
                    sensor2_filtered_z = data_match.group(13)

                    # Speichern der gefilterten Daten
                    row_data = [time_value, sensor1_filtered_x, sensor1_filtered_y, sensor1_filtered_z, sensor2_filtered_x, sensor2_filtered_y, sensor2_filtered_z]
                    
                    if validate_row(row_data):
                        # Write the data to the CSV file if validation passes
                        writer.writerow(row_data)
                        file.flush()
                        os.fsync(file.fileno())
                        print(f"Data written to CSV: {row_data}")
                    else:
                        print("Invalid data, skipping row.")
                else:
                    print("Time and sensor data could not be extracted")
            else:
                print("No data in buffer, waiting...")
                time.sleep(0.5)
except KeyboardInterrupt:
    print("Terminated by user.")
finally:
    ser.close()
    print("Serial connection closed.")
