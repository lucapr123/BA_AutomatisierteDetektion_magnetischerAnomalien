import sys
import rclpy
from rclpy.node import Node
import pandas as pd
from std_msgs.msg import String
import os
import time

class AnomalyPublisher(Node):

    def __init__(self):
        super().__init__('anomaly_publisher')
        
        self.get_logger().info("Initializing AnomalyPublisher...")

        self.csv_file_path = '/home/ghost/Magnetometer/Datenaufbereitung/Detektierte_Anomalien.csv'  # Pfad zu Ihrer CSV-Datei
        
        # Überprüfen Sie, ob die CSV-Datei existiert
        if not os.path.exists(self.csv_file_path):
            self.get_logger().error(f'CSV file does not exist: {self.csv_file_path}')
            return

        # Initialisieren Sie den Publisher
        self.publisher = self.create_publisher(String, 'Anomalieerkennung', 10)
        self.get_logger().info("Publisher initialized for Anomalieerkennung topic.")

        # Initialisieren Sie den letzten Zeilenzähler
        self.last_row_count = 0

        self.timer_period = 5.0  # Sekunden
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Timer initialized.")

    def timer_callback(self):
        try:
            # Laden Sie die CSV-Datei
            anomalies_df = pd.read_csv(self.csv_file_path)
            
            # Überprüfen Sie die Anzahl der Zeilen
            current_row_count = len(anomalies_df)
            
            if current_row_count > self.last_row_count:
                new_entries = anomalies_df.iloc[self.last_row_count:]
                for index, row in new_entries.iterrows():
                    anomaly_data = row.to_json()  # Konvertieren Sie jede Zeile in JSON-Format
                    msg = String()
                    msg.data = anomaly_data
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published anomaly data: {anomaly_data}')
                    time.sleep(1)  # Optional: kleine Verzögerung zwischen den Veröffentlichungen
                
                # Aktualisieren Sie den Zeilenzähler
                self.last_row_count = current_row_count
            else:
                self.get_logger().info('No new entries found.')

        except Exception as e:
            self.get_logger().error(f'Error while processing CSV data: {e}')
            self.emergency_restart()

    def emergency_restart(self):
        self.get_logger().error('Critical error occurred. Restarting the node...')
        time.sleep(2)  # Kleine Verzögerung vor dem Neustart
        os.execv(sys.executable, ['python3'] + sys.argv)

def main(args=None):
    rclpy.init(args=args)

    anomaly_publisher = None
    try:
        anomaly_publisher = AnomalyPublisher()
        rclpy.spin(anomaly_publisher)
    except Exception as e:
        print(f"Exception in main: {e}")
        # Hier könnte auch ein Neustart erfolgen, falls der Hauptteil des Codes fehlschlägt.
        AnomalyPublisher().emergency_restart()
    finally:
        # Shutdown, wenn anomaly_publisher erstellt wurde
        if anomaly_publisher:
            anomaly_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
