import rclpy
import time
import threading
from rclpy.node import Node
from custom_msgs.msg import ImagePair  # type: ignore
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import cv2
from flask import Flask, Response

latest_frame = None  # global frame buffer

class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.qos_profile = qos_profile_sensor_data
        self.subscription = self.create_subscription(ImagePair, '/stereo/image_pair', self.listener_callback, self.qos_profile)

    def listener_callback(self, msg):
        global latest_frame
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        id = msg.header.frame_id
        time_0 = time.time()
        try:
            np_arr_left = np.frombuffer(msg.left.data, dtype=np.uint8)
            frame_left = cv2.imdecode(np_arr_left, cv2.IMREAD_COLOR)

            np_arr_right = np.frombuffer(msg.right.data, dtype=np.uint8)
            frame_right = cv2.imdecode(np_arr_right, cv2.IMREAD_COLOR)

            if frame_left is None or frame_right is None:
                self.get_logger().error("Can't decode frame")
                return

            combined = np.hstack((frame_left, frame_right))
            latest_frame = combined  # update global frame

        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")

        delta_receive = round((time.time() - timestamp) * 1000, 2)
        delta_process = round((time.time() - time_0) * 1000, 2)
        self.get_logger().info(f"{id}, R: {delta_receive}ms, P: {delta_process}ms")


# MJPEG Flask server
app = Flask(__name__)

def generate_mjpeg():
    global latest_frame
    while True:
        if latest_frame is not None:
            ret, jpeg = cv2.imencode('.jpg', latest_frame)
            if not ret:
                continue
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.03)  # ~30 FPS

@app.route('/')
def index():
    return '<h1>Stereo Stream</h1><img src="/video_feed" width="100%">'

@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def flask_thread():
    app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)

def main(args=None):
    rclpy.init(args=args)

    ros_node = TestSubscriber()

    # Start Flask server in separate thread
    flask_threading = threading.Thread(target=flask_thread, daemon=True)
    flask_threading.start()

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
