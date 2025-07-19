import rclpy
import time
import threading
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, HistoryPolicy
import numpy as np
import cv2
from flask import Flask, Response

latest_frame = None

class ResultServer(Node):

    TN_RESULTS = "/output/cv_mono_ir_frame"
    QOS_DEPTH_SUB = 1

    def __init__(self):
        super().__init__("results_server_node")
        qos_profile = QoSProfile(depth=self.QOS_DEPTH_SUB)
        self.subscription = self.create_subscription(Image, self.TN_RESULTS, self.listener_callback, qos_profile)
        self.subscription
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        if msg is None:
            self.get_logger().error("Can't decode frame")
            return
        global latest_frame
        #time_0 = time.time()
        try:
            latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")
        #delta_receive = round((time.time() - timestamp) * 1000, 2)
        #delta_process = round((time.time() - time_0) * 1000, 2)
        #self.get_logger().debug(f"{id}, R: {delta_receive}ms, P: {delta_process}ms")


# MJPEG Flask server
app = Flask(__name__)

def generate_mjpeg():
    global latest_frame
    while True:
        if latest_frame is not None:
            _, buffer = cv2.imencode(".png", latest_frame)
            frame = buffer.tobytes()
            yield (b"--frame\r\n"
                   b"Content-Type: image/png\r\n\r\n" + frame + b"\r\n")
        time.sleep(0.03)  # ~30 FPS

@app.route('/')
def index():
    return "<h1>Monovision Image Recognition results</h1><img src=\"/video_feed\" width=\"80%\">"

@app.route("/video_feed")
def video_feed():
    return Response(generate_mjpeg(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

def flask_thread():
    app.run(host="0.0.0.0", port=8081, debug=False, threaded=True)

def main(args=None):
    rclpy.init(args=args)

    ros_node = ResultServer()

    # Start Flask server in separate thread
    flask_threading = threading.Thread(target=flask_thread, daemon=True)
    flask_threading.start()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
