import rclpy
from cv_bridge import CvBridge
from custom_msgs.msg import StereoIR
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

import torch

from cv_algorithms.methods_stereo_ir import *

"""
Class to create a Python node to compute and publish
image recognition algorithm on stereovision
"""
class NodeStereoIR(Node):
    """
    Define class costants
    """
    TN_STEREO = "/output/cv_stereo"
	# TODO
    TN_INPUT = "/TODO/your/topic/name"
    QOS_DEPTH_SUB = 1
    QOS_DEPTH_PUB = 5
    MODEL_PATH = "/home/ubuntu/ROS2_WSs/ws_sensing_rigs_v_two/src/cv_algorithms/cv_algorithms/models/classifier.pth"

    """
    Default class constructor
    """
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

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
        # TODO: compatible?
        qos_profile_sub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=self.QOS_DEPTH_SUB)
        # Publisher
        self.publisher_ = self.create_publisher(StereoIR, self.TN_STEREO, qos_profile_pub)
        # Subscriber
        self.subscription = self.create_subscription(CompressedImage, self.TN_INPUT, self.callback_subscribe, qos_profile_sub)
        self.subscription

    """
    Function to initialise image recognition model
    """
    def init_image_recog(self):
        self.bridge = CvBridge()
        self.device = None
        self.model = None
        # 1. Controllo disponibilità GPU/CPU
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if self.get_parameter("debug").get_parameter_value().bool_value:
            self.get_logger().info(f"Device utilizzato: {self.device}")
        if self.device.type == "cpu":
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("ATTENZIONE: GPU non disponibile, le performance potrebbero essere ridotte")
        # 2. Controllo presenza modello
        model_ok, model_msg = check_model(self.MODEL_PATH, self.device)
        if not model_ok:
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info(f"Modello non trovato: {model_msg}")
            quit()
        if self.get_parameter("debug").get_parameter_value().bool_value:
            self.get_logger().info(f"Modello trovato: {model_msg}")
        # Carica il modello
        try:
            self.model = EnhancedClassifier(num_classes=2).to(self.device)
            self.model.load_state_dict(torch.load(self.MODEL_PATH, map_location=self.device))
            self.model.eval()
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("Modello caricato correttamente")
        except Exception as e:
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info(f"Errore nel caricamento del modello: {str(e)}")
            quit()

    """
    """
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.init_param()
        self.init_var()
        self.init_image_recog()
        self.get_logger().info("Stereovision I.R. node configured!")
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
    def callback_publish(self, data):
        if (self.publisher_ is None) or (not self.publisher_.is_activated):
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("Data received, but inactive node!")
        else:
            message = StereoIR()
            message.timestamp = datetime.today().strftime("%Y-%m-%d %H:%M:%S")
            message.label = data
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info(f"Publishing: \"{message}\"")
            if self.get_parameter("save").get_parameter_value().bool_value:
                self.get_logger().info("Message saved to rosbag!")
            self.publisher_.publish(message)

    """
    Callback function to get input data
    """
    def callback_subscribe(self, msg):
        if (self.publisher_ is None) or (not self.publisher_.is_activated):
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("Frames received, but inactive node!")
        else:
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("Frames received!")
            lf = self.bridge.imgmsg_to_cv2(msg)
            rf = self.bridge.imgmsg_to_cv2(msg)
            out = self.analyze_frames(lf, rf)
            self.callback_publish(out)

    """
    Function that takes 2 frames of a stereo vision image
    and applies an image recognition algorithm given
    the selected model.
    Returns a string (what has been recognized) and a counter
    """
    def analyze_frames(self, left_frame, right_frame):
        if left_frame is None or right_frame is None:
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info("Frame non disponibile, riprovo...")
        # Pre-elaborazione
        try:
            depth = compute_depth_map_advanced(left_frame, right_frame)
            enhanced_left = enhanced_retinex(left_frame, depth)
        except Exception as e:
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info(f"Errore nella pre-elaborazione: {str(e)}")
        # Prepara il tensore
        try:
            input_tensor = torch.from_numpy(enhanced_left).float().permute(2, 0, 1).unsqueeze(0).to(device) / 255.0
        except Exception as e:
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info(f"Errore nella preparazione del tensore: {str(e)}")
        # Inferenza
        try:
            with torch.no_grad():
                outputs = model(input_tensor)
                _, predicted = torch.max(outputs, 1)
                is_scallop = predicted.item() == 1
                label = "Capesante" if is_scallop else "Altro"
        except Exception as e:
            if self.get_parameter("debug").get_parameter_value().bool_value:
                self.get_logger().info(f"Errore nell'inferenza: {str(e)}")
        return label

def main(args = None):
    rclpy.init(args = args)
    exectr = MultiThreadedExecutor()
    stereoir_node = NodeStereoIR("lc_stereoir_node")
    exectr.add_node(stereoir_node)
    try:
        exectr.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\nInterrupt detected, killing node...")
    finally:
        stereoir_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
