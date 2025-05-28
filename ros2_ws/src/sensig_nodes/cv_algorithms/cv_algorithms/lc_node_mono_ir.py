import rclpy
from cv_bridge import CvBridge
from custom_msgs.msg import MonoIR
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from cv_algorithms.methods_mono_ir import *

"""
Class to create a Python node to compute and publish
image recognition algorithm on monovision
"""
class NodeMonoIR(Node):
    """
    Define class costants
    """
    TN_MONO = "/output/cv_mono_ir"
	# TODO
    TN_INPUT = "/TODO/your/topic/name"
    QOS_DEPTH_SUB = 1
    QOS_DEPTH_PUB = 5
    CAMERA_ID = 0 # id of the camera (default is 0, if there's more than one camera change the integer)
    INFERENCE_INTERVAL = 2 # interval (in seconds) for capturing data
    CONF_THRESHOLD = 0.5 # threshold of confidence the inference must have in order to save
    COLOR_BOX_AND_TEXT = (0,225,255) # color of bounding boxes and text
    THICKNESS_BOX_AND_TEXT = 1 # thickness of lines and text
    FONT_SIZE = 2 # scale the font size
	# TODO
    MODEL_PATH = "/home/ubuntu/ROS2_WSs/ws_sensing_rigs_v_two/src/cv_algorithms/cv_algorithms/models/yolo11n.pt"

    """
    Default class constructor
    """
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

    """
    Function to initialize node's parameters
    """
    def init_param(self):
        # Initialize parameters
        param_descr_debug = ParameterDescriptor(description = "Prints additional debug informations.")
        param_descr_save = ParameterDescriptor(description = "Saves odometry messages on ROSbags.")
        self.declare_parameter("debug", False, param_descr_debug)
        self.declare_parameter("save", False, param_descr_save)

    """
    Function to initialize node's variables
    """
    def init_var(self):
        # Initialize QoS profiles
        qos_profile_pub = QoSProfile(depth=self.QOS_DEPTH_PUB)
		#TODO: compatible?
        qos_profile_sub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=self.QOS_DEPTH_SUB)
        # Publisher
        self.publisher_ = self.create_publisher(MonoIR, self.TN_MONO, qos_profile_pub)
        # Subscriber
        self.subscriber_ = self.create_subscription(CompressedImage, self.TN_INPUT, self.callback_subscribe, qos_profile_sub)
        self.subscriber_
        
    """
    Function to initialize image recognition variables
    """
    def init_image_recog(self):
        self.bridge = CvBridge()
        self.model = None
        self.prev_gray = None
        self.last_frameCheck_time = 0
        self.model = YOLO(self.MODEL_PATH,task="detect")

    """
    """
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.init_param()
        self.init_var()
        self.init_image_recog()
        self.get_logger().info("Monovision I.R. node configured!")
        self.status_param()
        return TransitionCallbackReturn.SUCCESS

    """
    """
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        return super().on_activate(state)

    """
    """
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        return super().on_deactivate(state)

    """
    """
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisher_)
        self.destroy_subscriber(self.subscriber_)
        return TransitionCallbackReturn.SUCCESS

    """
    """
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.publisher_)
        self.destroy_subscriber(self.subscriber_)
        return TransitionCallbackReturn.SUCCESS

    """
    Function that lists all parameters
    """
    def status_param(self):
        self.get_logger().info(f"debug:\t{self.get_parameter('debug').get_parameter_value().bool_value}")
        self.get_logger().info(f"save:\t{self.get_parameter('save').get_parameter_value().bool_value}")

    """
    Callback function to publish result
    """
    def callback_publish(self, ts, label, cd, coordinates):
        if (self.publisher_ is None) or (not self.publisher_.is_activated):
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("Data received, but inactive node!")
        else:
            message = MonoIR()
            message.timestamp = ts
            message.label = label
            message.confidence = cd
            message.box = coordinates
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info(f"Published:\n\t{ts}\n\t{label}\n\t{cd}\n\t{coordinates}")
            if self.get_parameter("save").get_parameter_value().bool_value:
                self.get_logger().info("Message saved to rosbag!")
            self.publisher_.publish(message)

    """
    Callback function to get input data
    """
    def callback_subscribe(self, msg):
        if (self.publisher_ is None) or (not self.publisher_.is_activated):
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("Frame received, but inactive node!")
        else:
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("Frame received!")
            frame = self.bridge.imgmsg_to_cv2(msg)
            out = self.analyze_frame(frame)
            if out:
                timestamp = out[0]
                name = out[1]
                for c in (range(2, len(out))):
                    self.callback_publish(timestamp, name, out[c][0], [out[c][1], out[c][2], out[c][3], out[c][4]])

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
                self.prev_gray = current_gray.copy()
                if self.get_parameter("debug").get_parameter_value().bool_value:
                    self.get_logger().info("First frame acquired!")
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
            # Reset timer for inference
            self.last_frameCheck_time = current_time

        except Exception as e:
            self.get_logger().error(e)
        finally:
            return log_data

def main(args = None):
    rclpy.init(args = args)
    exectr = MultiThreadedExecutor()
    monoir_node = NodeMonoIR("lc_monoir_node")
    exectr.add_node(monoir_node)
    try:
        exectr.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\nInterrupt detected, killing node...")
    finally:
        monoir_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
