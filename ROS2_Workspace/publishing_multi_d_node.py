# Publishen von Positionsdaten an UAV aus detektierten Anomalien

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

        self.csv_file_path = '/home/ghost/Magnetometer/Datenaufbereitung/Detektierte_Anomalien_GPS.csv'  # Path to the CSV file
        
        if not os.path.exists(self.csv_file_path):
            self.get_logger().error(f'CSV file does not exist: {self.csv_file_path}')
            return

        # Initialize the publisher
        self.publisher = self.create_publisher(String, 'Anomalien_GoTo_UAV', 10)
        self.get_logger().info("Publisher initialized for Anomalien_GoTo_UAV topic.")

        self.timer_period = 10.0  # Seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Timer initialized.")

        # Variables to store last published coordinates
        self.last_published_dms_n = None
        self.last_published_dms_e = None

    def timer_callback(self):
        try:
            # Load the CSV file and read the last row
            anomalies_df = pd.read_csv(self.csv_file_path)
            
            if not anomalies_df.empty:
                # Select the last row
                last_entry = anomalies_df.iloc[-1]
                
                # Extract the DMS-N and DMS-E columns
                dms_n = last_entry['DMS_N']
                dms_e = last_entry['DMS_E']
                
                # Check if the last entry has already been published
                if self.should_publish(dms_n, dms_e):
                    anomaly_data = last_entry[['DMS_N', 'DMS_E']].to_json()
                    msg = String()
                    msg.data = anomaly_data
                    self.publisher.publish(msg)
                    self.get_logger().info(f'Published new Anomalie GoTo Data for the UAV: {anomaly_data}')

                    # Update the last published values
                    self.last_published_dms_n = dms_n
                    self.last_published_dms_e = dms_e
                else:
                    self.get_logger().info('No significant change in anomaly data; not publishing.')

            else:
                self.get_logger().info('CSV file is empty.')

        except KeyError as e:
            self.get_logger().error(f"Error while processing CSV data: {e}. Available columns: {anomalies_df.columns.tolist()}")
            self.emergency_restart()

        except Exception as e:
            self.get_logger().error(f'Error while processing CSV data: {e}')
            self.emergency_restart()

    def should_publish(self, dms_n, dms_e):
        """
        Check if the current row should be published. Returns True if:
        - The coordinates haven't been published before
        - The 4th decimal digit in either DMS_N or DMS_E has changed
        """
        if self.last_published_dms_n is None or self.last_published_dms_e is None:
            # First time publishing, so publish this row
            return True

        # Get the 4th decimal digit for both current and last published values
        current_dms_n_4th = int(str(round(dms_n, 5)).split('.')[1][3])  # 4th decimal of DMS_N
        current_dms_e_4th = int(str(round(dms_e, 5)).split('.')[1][3])  # 4th decimal of DMS_E
        last_dms_n_4th = int(str(round(self.last_published_dms_n, 5)).split('.')[1][3])
        last_dms_e_4th = int(str(round(self.last_published_dms_e, 5)).split('.')[1][3])

        # Check if the 4th decimal of either coordinate has changed
        if current_dms_n_4th != last_dms_n_4th or current_dms_e_4th != last_dms_e_4th:
            return True
        else:
            return False

    def emergency_restart(self):
        self.get_logger().error('Critical error occurred. Restarting the node...')
        time.sleep(2)  # Small delay before restart
        os.execv(sys.executable, ['python3'] + sys.argv)

def main(args=None):
    rclpy.init(args=args)

    anomaly_publisher = None
    try:
        anomaly_publisher = AnomalyPublisher()
        rclpy.spin(anomaly_publisher)
    except Exception as e:
        print(f"Exception in main: {e}")
        # Restart if the main part fails
        AnomalyPublisher().emergency_restart()
    finally:
        # Shutdown if anomaly_publisher was created
        if anomaly_publisher:
            anomaly_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
