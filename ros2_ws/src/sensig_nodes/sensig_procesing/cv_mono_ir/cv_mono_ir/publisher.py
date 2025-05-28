import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from custom_msgs.msg import MonoIR
from rclpy.qos import QoSProfile, HistoryPolicy

from cv_mono_ir.functions_mono_ir import *

"""
Class to create a Python node to compute and publish
image recognition algorithm on monovision
"""
class MonoIRPub(Node):
    """
    Define class costants
    """
    TN_MONO = "/output/cv_mono"
	# TODO:
    TN_INPUT = "/TODO/your/topic/name"
    QOS_DEPTH_SUB = 1
    QOS_DEPTH_PUB = 5
    CAMERA_ID = 0 # id of the camera (default is 0, if there's more than one camera change the integer)
    INFERENCE_INTERVAL = 2 # interval (in seconds) for capturing data
    CONF_THRESHOLD = 0.5 # threshold of confidence the inference must have in order to save
    COLOR_BOX_AND_TEXT = (0,225,255) # color of bounding boxes and text
    THICKNESS_BOX_AND_TEXT = 1 # thickness of lines and text
    FONT_SIZE = 2 # scale the font size
	# TODO: put your own
    MODEL_PATH = "/home/admin/Desktop/ros2_jazzy_workspaces/ws_sensing_rigs_v_one/src/cv_mono_ir/cv_mono_ir/models/yolo11n.pt"

    """
    Default class constructor
    """
    def __init__(self):
        self.init_var()
        self.init_image_recog()
        # DEBUG
        self.get_logger().info("Node initialized!")

    """
    Function to initialize node's variables
    """
    def init_var(self):
        super().__init__("cv_mono_ir_pub")
        # Publisher
        qos_profile_pub = QoSProfile(depth=self.QOS_DEPTH_PUB)
        self.publisher_ = self.create_publisher(MonoIR, self.TN_MONO, qos_profile_pub)
        # Subscriber
		#TODO: compatible with your code?
        qos_profile_sub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=self.QOS_DEPTH_SUB)
        self.subscription = self.create_subscription(Image, self.TN_INPUT, self.callback_subscribe, qos_profile_sub)
        self.subscription
        # Mono vison variables
        self.bridge = CvBridge()
        self.model = None
        self.prev_gray = None
        self.last_frameCheck_time = 0

    """
    Function to initialize image recognition variables
    """
    def init_image_recog(self):
        # Load model
        self.model = YOLO(self.MODEL_PATH,task="detect")

    """
	Function to analyze a frame
    """
    def analyze_frame(self, frame):
        # Lista per il log
        log_data = []
        try:
            current_time = time.time()
            # Process the current frame, detect movement and save in prev_gray the processed current frame (to avoid processing 2 times the same frame)
            current_gray = gray_and_blur(frame)
            if(self.prev_gray == None):
                print("First frame acquired")
                self.prev_gray = current_gray.copy()
                return log_data
            movement_detected = processed_movement_detection(self.prev_gray, current_gray)
            if movement_detected:
                # Inference
                results = self.model(frame)[0] #calls the model and performs inference on frame, [0] because it returns a list of results (in this case made of only one item)
                # Create copy of original image to plot on
                annotated_frame = frame.copy()
                # Filter confidence > CONF_THRESHOLD
                confident_boxes = [
                    box for box in results.boxes if box.conf[0] > CONF_THRESHOLD
                ]
                if confident_boxes:
                    # Save annotated frame
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S") #different format from time.time(), needed for putting human-readable timestamps
                    log_data.append(timestamp)
                    log_data.append("Crab")
                    # Draw only "confident" boxes
                    for box in confident_boxes:
                        annotate_frame(self.model, annotated_frame, box, self.COLOR_BOX_AND_TEXT, self.FONT_SIZE, self.THICKNESS_BOX_AND_TEXT)
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        log_data.append([float(box.conf[0]), x1, y1, x2, y2])
                    """
                    # Set detection information dictionary for this frame
                    detection_info = {
                        "filename": filename,
                        "timestamp": timestamp,
                        "detections": []
                    }
                    """
            # Reset timer for inference
            self.last_frameCheck_time = current_time

        except Exception as e:
            print(e)
        finally:
            return log_data

    """
    Callback function to publish result
    """
    def callback_publish(self, ts, label, cd, coordinates):
        message = MonoIR()
        message.timestamp = ts
        message.label = label
        message.confidence = cd
        message.box = coordinates
        self.publisher_.publish(message)
        # DEBUG
        self.get_logger().info(f"Publishing: \"{message}\"")

    """
    Callback function to get input data
    """
    def callback_subscribe(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        out = self.analyze_frame(frame)
        if out:
            timestamp = out[0]
            name = out[1]
            for c in (range(2, len(out))):
                self.callback_publish(timestamp, name, out[c][0], [out[c][1], out[c][2], out[c][3], out[c][4]])

def main(args=None):
    rclpy.init(args=args)
    cv_node = MonoIRPub()
    rclpy.spin(cv_node)
    cv_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
