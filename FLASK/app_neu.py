import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
from flask import Flask, render_template, Response, jsonify
import os
import threading
from flask import Response, json

app = Flask(__name__)

# Lists to hold image data for normal and Gesamt plots
image_data_list_normal = [None] * 4  # For /plot_topic_1 to /plot_topic_4
image_data_list_gesamt = [None] * 6  # For /plot_topic_10 to /plot_topic_15
anomaly_data_list = []  # List for anomaly data
lock = threading.Lock()

# Flag to ensure nodes are initialized only once
nodes_initialized = False

class ImageSubscriber(Node):
    def __init__(self, topic_name, index, is_gesamt=False):
        node_name = f'image_subscriber_{"gesamt_" if is_gesamt else ""}{index}'
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.index = index
        self.is_gesamt = is_gesamt
        self.last_received_time = time.time()

    def listener_callback(self, msg):
        global image_data_list_normal, image_data_list_gesamt
        current_time = time.time()

        # Process the image every 4 seconds
        if current_time - self.last_received_time >= 4:
            topic_type = "Gesamt" if self.is_gesamt else "Normal"
            self.get_logger().info(f'Processing image from {topic_type} topic {self.index + 1}')
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                _, buffer = cv2.imencode('.png', cv_image)
                image_bytes = buffer.tobytes()

                with lock:
                    if self.is_gesamt:
                        image_data_list_gesamt[self.index] = image_bytes
                    else:
                        image_data_list_normal[self.index] = image_bytes

                self.last_received_time = current_time
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")
        else:
            self.get_logger().info(
                f'Skipping image from {"Gesamt" if self.is_gesamt else "Normal"} topic {self.index + 1}, only {current_time - self.last_received_time:.2f} seconds since last processing.')

class AnomalySubscriber(Node):
    def __init__(self, topic_name):
        super().__init__('anomaly_subscriber')
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global anomaly_data_list
        anomaly_data = msg.data  # Receive anomaly data
        self.get_logger().info(f'Received anomaly data: {anomaly_data}')
        with lock:
            anomaly_data_list.append(anomaly_data)

# Function to start ROS 2 nodes
def start_ros2_nodes():
    global nodes_initialized
    if not nodes_initialized:
        rclpy.init()

        # Define normal and gesamt topics
        normal_topics = [f'plot_topic_{i+1}' for i in range(4)]  # /plot_topic_1 to /plot_topic_4
        gesamt_topics = [f'plot_topic_{i+10}' for i in range(6)]  # /plot_topic_10 to /plot_topic_15

        # Create ImageSubscriber nodes for normal and gesamt topics
        normal_nodes = [ImageSubscriber(topic, i, is_gesamt=False) for i, topic in enumerate(normal_topics)]
        gesamt_nodes = [ImageSubscriber(topic, i, is_gesamt=True) for i, topic in enumerate(gesamt_topics)]

        # Create AnomalySubscriber node
        anomaly_node = AnomalySubscriber('Anomalieerkennung')

        # Combine all nodes
        all_nodes = normal_nodes + gesamt_nodes + [anomaly_node]

        # Create executor and add nodes
        executor = MultiThreadedExecutor()
        for node in all_nodes:
            executor.add_node(node)

        nodes_initialized = True  # Prevent reinitialization

        try:
            executor.spin()
        finally:
            executor.shutdown()
            for node in all_nodes:
                node.destroy_node()
            rclpy.shutdown()

# Flask Routes

@app.route('/welcome')
def welcome():
    return render_template('welcome.html')

@app.route('/index')
def index():
    return render_template('index.html')

@app.route('/Terminal')
def terminal():
    global anomaly_data_list
    with lock:
        data = anomaly_data_list.copy()
    return render_template('terminal.html', anomaly_data=data)

# Generator for normal plot feeds
def generate_normal(image_index):
    global image_data_list_normal
    while True:
        with lock:
            image_data = image_data_list_normal[image_index]
        if image_data is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/png\r\n\r\n' + image_data + b'\r\n\r\n')
        time.sleep(1)  # Update every second

@app.route('/plot_feed/<int:image_index>')
def plot_feed(image_index):
    if image_index < 0 or image_index >= len(image_data_list_normal):
        return "Invalid image index for normal plots.", 404
    return Response(generate_normal(image_index), mimetype='multipart/x-mixed-replace; boundary=frame')

# Generator for gesamt plot feeds
def generate_gesamt(image_index):
    global image_data_list_gesamt
    while True:
        with lock:
            image_data = image_data_list_gesamt[image_index]
        if image_data is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/png\r\n\r\n' + image_data + b'\r\n\r\n')
        time.sleep(300)  # Update every 5 minutes

@app.route('/gesamt_feed/<int:image_index>')
def gesamt_feed(image_index):
    if image_index < 0 or image_index >= len(image_data_list_gesamt):
        return "Invalid image index for gesamt plots.", 404
    return Response(generate_gesamt(image_index), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/anomaly_data')
def get_anomaly_data():
    global anomaly_data_list
    with lock:
        data = anomaly_data_list.copy()
    response = jsonify({'anomaly_data': data})
    return response

@app.route('/Gesamt')
def gesamt():
    return render_template('gesamt.html')

if __name__ == '__main__':
    # Start ROS 2 nodes in a separate daemon thread
    threading.Thread(target=start_ros2_nodes, daemon=True).start()
    
    # Start Flask application
    app.run(host='0.0.0.0', port=5008, debug=True, threaded=True)
