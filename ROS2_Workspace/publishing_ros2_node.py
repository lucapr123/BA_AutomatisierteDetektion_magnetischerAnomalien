# Publishe alle Sensordaten (Rohdaten, Totalfeld, Differenzen, Differenzen_Betrag, Integrierte_Differenz, Integrierte_Differenz_Betrag)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import pandas as pd

class SensorDataPublisher(Node):
    def __init__(self):
        super().__init__('sensor_data_publisher')

        # Publisher für verschiedene Topics
        self.sensor_data_pub = self.create_publisher(Vector3, '/Mag_sensor_data', 10)
        self.gradient_pub = self.create_publisher(Vector3, '/Mag_gradient', 10)
        self.integrated_gradient_pub = self.create_publisher(Vector3, '/Mag_integrated_gradient', 10)
        self.total_field_pub = self.create_publisher(Float64, '/Mag_total_field', 10)
        self.total_gradient_pub = self.create_publisher(Float64, '/Mag_total_gradient', 10)
        self.total_integrated_gradient_pub = self.create_publisher(Float64, '/Mag_total_integrated_gradient', 10)

        # Lade die CSV-Dateien (wird in jeder Schleife neu geladen, um immer den neuesten Eintrag zu erhalten)
        self.messung_file_path = '/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/Messung_aktuell.csv'
        self.gradients_file_path = '/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/Messung_aktuell/gradients.csv'
        self.integrated_gradients_file_path = '/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/Messung_aktuell/IntegratedTotalGradients.csv'

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        # Lade die neuesten Einträge aus den CSV-Dateien
        try:
            # Rohdaten
            messung_data = pd.read_csv(self.messung_file_path)
            sensor_1_row = messung_data.iloc[-1]  # Letzter Eintrag

            # Gradienten
            gradients_data = pd.read_csv(self.gradients_file_path)
            gradient_row = gradients_data.iloc[-1]  # Letzter Eintrag

            # Integrierte Gradienten
            integrated_gradients_data = pd.read_csv(self.integrated_gradients_file_path)
            integrated_gradient_row = integrated_gradients_data.iloc[-1]  # Letzter Eintrag
        except Exception as e:
            self.get_logger().error(f'Fehler beim Laden der CSV-Dateien: {e}')
            return

        # Rohdaten (sensor_data) aus der Messung_aktuell.csv (Sensor 1: Achse x, y, z)
        sensor_data_msg = Vector3()
        sensor_data_msg.x = sensor_1_row['Sensor 1: Achse x']
        sensor_data_msg.y = sensor_1_row['Sensor 1: Achse y']
        sensor_data_msg.z = sensor_1_row['Sensor 1: Achse z']

        # Gradient aus gradients.csv
        gradient_msg = Vector3()
        gradient_msg.x = gradient_row['dx']
        gradient_msg.y = gradient_row['dy']
        gradient_msg.z = gradient_row['dz']

        # Integrierter Gradient aus integratedtotalgradients.csv
        integrated_gradient_msg = Vector3()
        integrated_gradient_msg.x = integrated_gradient_row['Integrated dx']
        integrated_gradient_msg.y = integrated_gradient_row['Integrated dy']
        integrated_gradient_msg.z = integrated_gradient_row['Integrated dz']

        # Berechne Totalfeld (Magnitude des Vektors)
        total_field_value = Float64()
        total_field_value.data = (sensor_data_msg.x**2 + sensor_data_msg.y**2 + sensor_data_msg.z**2)**0.5

        # Berechne Totalgradient
        total_gradient_value = Float64()
        total_gradient_value.data = (gradient_msg.x**2 + gradient_msg.y**2 + gradient_msg.z**2)**0.5

        # Berechne Total integrierter Gradient
        total_integrated_gradient_value = Float64()
        total_integrated_gradient_value.data = (integrated_gradient_msg.x**2 + integrated_gradient_msg.y**2 + integrated_gradient_msg.z**2)**0.5

        # Veröffentliche die Nachrichten auf verschiedenen Topics
        self.sensor_data_pub.publish(sensor_data_msg)
        self.gradient_pub.publish(gradient_msg)
        self.integrated_gradient_pub.publish(integrated_gradient_msg)
        self.total_field_pub.publish(total_field_value)
        self.total_gradient_pub.publish(total_gradient_value)
        self.total_integrated_gradient_pub.publish(total_integrated_gradient_value)

        self.get_logger().info(f'Sensor data published: {sensor_data_msg}')
        self.get_logger().info(f'Gradient published: {gradient_msg}')
        self.get_logger().info(f'Integrated gradient published: {integrated_gradient_msg}')
        self.get_logger().info(f'Total field: {total_field_value.data}, Total gradient: {total_gradient_value.data}, Total integrated gradient: {total_integrated_gradient_value.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
