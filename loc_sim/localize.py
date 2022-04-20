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

        # setup marker dictionary
        self.marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        
        # setup cv bridge
        self.cv_bridge = CvBridge()

        # setup subscriber
        self.sub = self.create_subscription(
            Image,
            '/overhead_cam/image_raw',
            self.image_callback,
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )

        #setup publisher
        # self.image_pub = self.create_publisher(
        #     Image,
        #     'vis',
        #     rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        # )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            'pose',
            rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value
        )


    def image_callback(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.marker_dict)

        if not corners:
            # no markers, early terminate
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, K, D)
        ids_ = ids.flatten().tolist()
        
        try:
            origin_ind = ids_.index(0)
            robot_ind = ids_.index(4)
        except ValueError:
            return

        origin_to_cam = transformation(rvecs[origin_ind], tvecs[origin_ind])
        robot_to_cam = transformation(rvecs[robot_ind], tvecs[robot_ind])

        robot_to_origin = np.linalg.inv(origin_to_cam)@robot_to_cam

        position = robot_to_origin[:,3]
        heading = np.arctan2(robot_to_origin[1,0],robot_to_origin[0,0])
        
        rot = Rotation.from_matrix(robot_to_origin[:3,:3])
        quat = rot.as_quat()

        # visualize
        # img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        # for i in range(len(rvecs)):
        #     cv2.drawFrameAxes(img, K, D, rvecs[i], tvecs[i], 0.1)
        #     org =  (corners[robot_ind][0,0,:] + corners[robot_ind][0,2,:])/2 + [50,50]
        #     cv2.putText(img, '({:.2f}, {:.2f}, {:.0f})'.format(*pose, np.degrees(heading)), org.astype(int), fontFace=cv2.FONT_HERSHEY_SIMPLEX, color=(0,0,255), fontScale=1, thickness=2)

        # out_img = self.cv_bridge.cv2_to_imgmsg(img)
        # self.image_pub.publish(out_img)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose)
        

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode(node_name='localization')

    rclpy.spin(node)
    rclpy.shutdown()