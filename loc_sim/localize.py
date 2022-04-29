from logging.handlers import RotatingFileHandler
import cv2
import numpy as np
import rclpy
import rclpy.qos
import rclpy.node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import yaml

# hardcoded parameters, may also read from camera_info
K = np.array([[1296, 0, 1296.5],[0, 1296,  1296.5],[0 ,0, 1]])
D = np.zeros((1,4))

def transformation(rvec, tvec):
    T = np.eye(4)
    R, _ = cv2.Rodrigues(rvec)
    T[:3,:3] = R
    T[:3,3] = tvec
    return T

def rvec2quat(rvec):
    angle = np.linalg.norm(rvec)
    return np.hstack([rvec/angle*np.sin(angle/2), np.cos(angle/2)])

class LocalizationNode(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # declare ROS parameters
        self.declare_parameter('config_yaml')
        
        # load config file
        self.config_yaml = self.get_parameter('config_yaml')
        if self.config_yaml is None:
            self.get_logger().error('Missing configuration YAML.')
            self.destroy_node()

        self._load_config()

        # setup marker dictionary
        self.marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        
        # setup cv bridge
        self.cv_bridge = CvBridge()

        # setup subscriber
        self.subs = list()
        
        for camera in self.config['cameras']:
            self.subs.append(self.create_subscription(
                Image,
                camera['topic'],
                self.image_callback,
                rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
            ))

        # setup publishers
        self.pubs = dict()
        
        for marker in self.config['roaming_markers']:
            self.pubs[marker['id']] = self.create_publisher(
                PoseStamped,
                marker['topic'],
                rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
            )

        # setup timer
        self.timer = self.create_timer(0.2, self.timer_callback)

    def _load_config(self):
        '''Load markers from config YAML file.'''

        self.fixed_markers = dict()

        with open(self.config_yaml.value, 'r') as f:
            self.config = yaml.load(f)

        for marker in self.config['fixed_markers']:
            position = marker['position']
            orientation = marker['orientation']
            R = Rotation.from_euler('xyz', [orientation['x'], orientation['y'], orientation['z']])
            t = -np.array([position['x'], position['y'], position['z']])
            T = np.eye(4)
            T[:3,:3] = R.as_matrix()
            T[:3,3] = t
            self.fixed_markers[marker['id']] = T

        self.roaming_markers = dict()  # keys: marker id, value: pose in [x,y,z,qx,qy,qz,qw]

        for marker in self.config['roaming_markers']:
            self.roaming_markers[marker['id']] = None
        

    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.marker_dict)

        if not corners:
            # no markers, early terminate
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.config['marker_size'], K, D)
        ids_ = ids.flatten().tolist()
        
        fixed_inds = [] # indices corresponding to fixed markers
        roam_inds = [] # indices corresponding to roaming markers

        for i, id_ in enumerate(ids_):
            if id_ in self.fixed_markers.keys():
                fixed_inds.append(i)
            
            if id_ in self.roaming_markers.keys():
                roam_inds.append(i)
                
        # just pick first fixed
        fixed_ind = fixed_inds[0]
        origin_to_fixed = self.fixed_markers[ids_[fixed_ind]]
        fixed_to_cam = transformation(rvecs[fixed_ind], tvecs[fixed_ind])
        origin_to_cam = fixed_to_cam @ origin_to_fixed

        # iterate over roaming markers
        for roam_ind in roam_inds:
            robot_to_cam = transformation(rvecs[roam_ind], tvecs[roam_ind])

            robot_to_origin = np.linalg.inv(origin_to_cam)@robot_to_cam

            position = robot_to_origin[:,3]
            
            rot = Rotation.from_matrix(robot_to_origin[:3,:3])
            quat = rot.as_quat()

            self.roaming_markers[ids_[roam_ind]] = [
                position[0],
                position[1],
                position[2],
                quat[0],
                quat[1],
                quat[2],
                quat[3]
            ]

    def timer_callback(self):
        for id_, data in self.roaming_markers.items():
            if data is not None:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = data[0]
                pose.pose.position.y = data[1]
                pose.pose.position.z = data[2]
                pose.pose.orientation.x = data[3]
                pose.pose.orientation.y = data[4]
                pose.pose.orientation.z = data[5]
                pose.pose.orientation.w = data[6]
                
                self.pubs[id_].publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode(node_name='localization')

    rclpy.spin(node)
    rclpy.shutdown()