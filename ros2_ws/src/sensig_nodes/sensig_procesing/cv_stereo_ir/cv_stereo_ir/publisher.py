import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from custom_msgs.msg import StereoIR
from rclpy.qos import QoSProfile, HistoryPolicy

import cv2
import torch
from datetime import datetime, timedelta
import numpy as np

from cv_stereo_ir.stereo_capture import StereoVideoProcessor
from cv_stereo_ir.models import EnhancedClassifier
from cv_stereo_ir.image_processing import compute_depth_map_advanced, enhanced_retinex
from cv_stereo_ir.inference import *

"""
Class to create a Python node to compute and publish
image recognition algorithm on stereovision
"""
class StereoIRPub(Node):
    """
    Define class costants
    """
    TN_STEREO = "/output/cv_stereo"
	# TODO:
    TN_INPUT = "/TODO/your/topic/name"
    QOS_DEPTH_SUB = 1
    QOS_DEPTH_PUB = 5
	# TODO: put your own
    MODEL_PATH = ""

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
        super().__init__("cv_stereo_ir_pub")
        # Publisher
        qos_profile_pub = QoSProfile(depth=self.QOS_DEPTH_PUB)
        self.publisher_ = self.create_publisher(StereoIR, self.TN_STEREO, qos_profile_pub)
        # Subscriber
		#TODO: compatible with your code?
        qos_profile_sub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=self.QOS_DEPTH_SUB)
        self.subscription = self.create_subscription(Image, self.TN_INPUT, self.callback_subscribe, qos_profile_sub)
        self.subscription
        # Image recognition variables
        self.bridge = CvBridge()
        self.device = None
        self.model = None

    """
    Function to initialise image recognition model
    """
    def init_image_recog(self):
        # 1. Controllo disponibilità GPU/CPU
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # DEBUG
        self.get_logger().info(f"Device utilizzato: {self.device}")
        if self.device.type == "cpu":
            # DEBUG
            self.get_logger().info("ATTENZIONE: GPU non disponibile, le performance potrebbero essere ridotte")
        # 2. Controllo presenza modello
        model_ok, model_msg = check_model(self.MODEL_PATH, self.device)
        if not model_ok:
            # DEBUG
            self.get_logger().info(f"Modello non trovato: {model_msg}")
            quit()
        # DEBUG
        self.get_logger().info(f"Modello trovato: {model_msg}")
        # Carica il modello
        try:
            self.model = EnhancedClassifier(num_classes=2).to(self.device)
            self.model.load_state_dict(torch.load(self.MODEL_PATH, map_location=self.device))
            self.model.eval()
            # DEBUG
            self.get_logger().info("Modello caricato correttamente")
        except Exception as e:
            # DEBUG
            self.get_logger().info(f"Errore nel caricamento del modello: {str(e)}")
            quit()

    """
    Callback function to publish result
    """
    def callback_publish(self, data):
        message = StereoIR()
        message.timestamp = datetime.today().strftime("%Y-%m-%d %H:%M:%S")
        message.label = data
        self.publisher_.publish(message)
        # DEBUG
        self.get_logger().info(f"Publishing: \"{message}\"")

    """
    Callback function to get input data
    """
    def callback_subscribe(self, msg):
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
            # DEBUG
            self.get_logger().info(f"Frame non disponibile, riprovo...")
        # Pre-elaborazione
        try:
            depth = compute_depth_map_advanced(left_frame, right_frame)
            enhanced_left = enhanced_retinex(left_frame, depth)
        except Exception as e:
            # DEBUG
            self.get_logger().info(f"Errore nella pre-elaborazione: {str(e)}")
        # Prepara il tensore
        try:
            input_tensor = torch.from_numpy(enhanced_left).float().permute(2, 0, 1).unsqueeze(0).to(device) / 255.0
        except Exception as e:
            # DEBUG
            self.get_logger().info(f"Errore nella preparazione del tensore: {str(e)}")
        # Inferenza
        try:
            with torch.no_grad():
                outputs = model(input_tensor)
                _, predicted = torch.max(outputs, 1)
                is_scallop = predicted.item() == 1
                label = "Capesante" if is_scallop else "Altro"
        except Exception as e:
            # DEBUG
            self.get_logger().info(f"Errore nell'inferenza: {str(e)}")
        return label

def main(args=None):
    rclpy.init(args=args)
    cv_node = StereoIRPub()
    rclpy.spin(cv_node)
    cv_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
