import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from custom_msgs.msg import Odometry
from rclpy.qos import QoSProfile, HistoryPolicy

from cv_stereo_odometry.odometry_functions import *

"""
Class to create a Python node to compute and publish
odometry data
"""
class StereoOdometryPub(Node):
    """
    Define class costants
    """
    TN_ODOMETRY = "/output/cv_odometry"
	# TODO:
    TN_INPUT = "/TODO/your/topic/name"
    QOS_DEPTH_SUB = 1
    QOS_DEPTH_PUB = 5
    FRAME_NUMBER = 1
    P0 = [[1.4071030581737341e+03, 0.0, 9.6464705146844085e+02, 0.0], [0.0, 1.4103004488655147e+03, 5.4965615887916658e+02, 0.0], [0.0, 0.0, 1.0, 0.0]]
    P1 = [[1.4057201280175375e+03, 0.0, 9.7297533642744793e+02, 1.7286159877160915e+02], [0.0, 1.4096343950827527e+03, 5.5423157044647405e+02, 4.1078470033654471e+00], [0.0, 0.0, 1.0, 1.7567302829839208e-03]]

    """
    Default class constructor
    """
    def __init__(self):
        self.init_var()
        self.init_odometry()
        # DEBUG
        self.get_logger().info("Node initialized!")

    """
    Function to initialize node's variables
    """
    def init_var(self):
        super().__init__("cv_stereo_odo_pub")
        # Publisher
        qos_profile_pub = QoSProfile(depth=self.QOS_DEPTH_PUB)
        self.publisher_ = self.create_publisher(Odometry, self.TN_ODOMETRY, qos_profile_pub)
        # Subscriber
		#TODO: compatible with your code?
        qos_profile_sub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=self.QOS_DEPTH_SUB)
        self.subscription = self.create_subscription(Image, self.TN_INPUT, self.callback_subscribe, qos_profile_sub)
        self.subscription

    """
    Function to initialise odometry model
    """
    def init_odometry(self):
        self.left_img_prec = None

    """
    Callback function to publish result
    """
    def callback_publish(self, data):
        message = Odometry()
        message.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        message.translation = [data[0][3], data[1][3], data[2][3]]
        message.rotation = [data[0][0], data[0][1], data[0][2], data[1][0], data[1][1], data[1][2], data[2][0], data[2][1], data[2][2]]
        self.publisher_.publish(message)
        # DEBUG
        self.get_logger().info(f"Publishing: \"{message}\"")
        self.get_logger().info(f"")

    """
    Callback function to get input data
    """
    def callback_subscribe(self, msg):
        lf = self.bridge.imgmsg_to_cv2(msg)
        rf = self.bridge.imgmsg_to_cv2(msg)
        out = self.visual_odometry(lf, rf)
        if out:
            self.callback_publish(out)

    """
    """
    def visual_odometry(left_img, right_img, stereo_matcher="bm", detector= "orb", matcher="BF", filter_threshold=None, mask= True, verbose=False, plot=False):
        if self.left_img_prec is None:
            self.left_img_prec = left_img
            #DEBUG
            self.get_logger().info("First left image acquired!")
            return []

        k_left,r_left,t_left = decomposeProjectionMatrix(self.P0)
        trajectory=np.zeros((self.FRAME_NUMBER, 3,4))
        T_tot=np.eye(4)
        trajectory[0]=(T_tot[:3,:])

        depth= stereo_to_depth(left_img, right_img, self.P0, self.P1)
        kp0,des0 = extract_features(left_img_prec)
        kp1,des1 = extract_features(left_img)
        self.left_img_prec = left_img
        unmatched_features= match_features(des0,des1)
        if filter_threshold is not None:
            matched_features=filter_matches(unmatched_features, filter_threshold)
        else:
            matched_features= unmatched_features #problem : list of 2-element list, ransac doesn't want it
        rvec,tvec= estimate_motion(kp0,kp1,k_left, matched_features, depth)
        #computing the transformation matrix [rvec| tvec]
        T_mat= np.hstack([rvec,tvec])
        #computing the homogeneous transformation matrix, so then I can invert it
        T_mat=np.vstack([T_mat, [0,0,0,1]])
        #inverting the transformation matrix
        T_mat=np.linalg.inv(T_mat)
        T_tot = T_tot.dot(T_mat)
        #appending T_tot into trajectory
        trajectory[i+1, :, :] = T_tot[:3][:]
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    cv_node = StereoOdometryPub()
    rclpy.spin(cv_node)
    cv_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
