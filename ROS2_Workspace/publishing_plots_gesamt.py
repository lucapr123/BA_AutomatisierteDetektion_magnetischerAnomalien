# Publishen der Gesamt-Plots nach Beenden der Messung
import sys
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class PlotPublisher(Node):

    def __init__(self):
        super().__init__('plot_publisher')
        
        self.get_logger().info("Initializing PlotPublisher...")

        self.bridge = CvBridge()
        self.image_directory = '/home/ghost/Magnetometer/Datenerfassung/Messung_aktuell/Messung_aktuell/Gesamt'
        
        # Überprüfen Sie, ob das Bildverzeichnis existiert
        if not os.path.exists(self.image_directory):
            self.get_logger().error(f'Image directory does not exist: {self.image_directory}')
            return

        # Sammeln Sie die Bilddateien
        self.image_files = sorted([f for f in os.listdir(self.image_directory) if f.endswith('.png')])
        
        # Überprüfen Sie, ob es Bilder im Verzeichnis gibt
        if not self.image_files:
            self.get_logger().error('No .png images found in the directory.')
            return

        self.get_logger().info(f'Found {len(self.image_files)} images.')

        # Initialisieren Sie Publisher für jedes Bild
        self.image_publishers = {}
        for i, image_file in enumerate(self.image_files):
            topic_name = f'plot_topic_{i+10}'
            self.image_publishers[topic_name] = {
                'publisher': self.create_publisher(Image, topic_name, 10),
                'image_file': image_file,
                'image_index': i
            }
            self.get_logger().info(f'Created publisher for {topic_name} with image {image_file}.')

        self.timer_period = 2.0  # 10 Sekunden zwischen den Publikationen
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info("Timer initialized.")

        self.publish_count = 0  # Zählt, wie oft der Timer aufgerufen wurde
        self.max_publish_count = len(self.image_files)  # Maximale Anzahl der Publikationen basierend auf den Bildern

    def timer_callback(self):
        self.get_logger().info(f'Publish count: {self.publish_count}, Max publish count: {self.max_publish_count}')
        if self.publish_count >= self.max_publish_count:
            self.get_logger().info('All images have been published, shutting down node.')
            rclpy.shutdown()  # Beendet die ROS2-Node
            return
        
        # Publizieren Sie das nächste Bild
        data = list(self.image_publishers.values())[self.publish_count]
        image_file = data['image_file']
        publisher = data['publisher']
        
        current_image_path = os.path.join(self.image_directory, image_file)

        try:
            # Überprüfen Sie, ob die Datei existiert und nicht leer ist
            if not os.path.exists(current_image_path) or os.path.getsize(current_image_path) == 0:
                self.get_logger().error(f'Image file does not exist or is empty: {current_image_path}')
                return

            self.get_logger().info(f'Attempting to publish image {current_image_path}')

            # Bild laden
            cv_image = cv2.imread(current_image_path)

            if cv_image is None or cv_image.size == 0:
                self.get_logger().error(f'Failed to read image or image is empty: {current_image_path}')
                return

            # In ROS Image-Nachricht konvertieren
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # Nachricht veröffentlichen
            publisher.publish(ros_image)
            self.get_logger().info(f'Successfully published image {image_file}')

            self.publish_count += 1  # Erhöhen Sie die Anzahl der veröffentlichten Bilder

        except Exception as e:
            self.get_logger().error(f'Error while processing {image_file}: {e}')
            self.publish_count += 1  # Fehlerhaften Bild überspringen

    def emergency_restart(self):
        self.get_logger().error('Critical error occurred. Restarting the node...')
        time.sleep(2)  # Kleine Verzögerung vor dem Neustart
        os.execv(sys.executable, ['python3'] + sys.argv)

def main(args=None):
    rclpy.init(args=args)

    plot_publisher = PlotPublisher()
    executor = SingleThreadedExecutor()
    executor.add_node(plot_publisher)

    try:
        # Spin den Executor, bis die Node heruntergefahren wird
        executor.spin()  # Dies wird beendet, wenn rclpy.shutdown() aufgerufen wird
    except Exception as e:
        print(f"Exception in main: {e}")
    finally:
        executor.shutdown()  # Executor herunterfahren
        plot_publisher.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
