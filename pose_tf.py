#!/usr/bin/env python3
import rclpy
import sys
import cv2
from cv2 import aruco
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import time
import sys
from threading import Thread



def calculate_rectangle_area(coordinates):
    x1, y1 = coordinates[0]
    x2, y2 = coordinates[1]
    x3, y3 = coordinates[2]
    x4, y4 = coordinates[3]

    width = max(abs(x2 - x1), abs(x4 - x3), abs(y2 - y1), abs(y4 - y3))

    area = 0.5 * abs((x1*y2 + x2*y3 + x3*y4 + x4*y1) - (y1*x2 + y2*x3 + y3*x4 + y4*x1))

    return area, width
    
def detect_aruco(image):
   
    aruco_area_threshold = 1500
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    size_of_aruco_m = 0.15
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
    ids1= [] 
    ids_list=[]
    corners2=[]
    corners1=[]
    tvec_list=[]
    tvec_l=[]
    rotation_matrix_list=[]

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is None:
        return [], [], [], [], []
    
    for i in range(len(ids)):

        coordinates = corners[i][0]
        area, width = calculate_rectangle_area(coordinates)

        if area > aruco_area_threshold:
            width_aruco_list.append(width)
            ids_list.append(ids[i])
            ids1 = np.vstack([arr.squeeze() for arr in ids_list])
            ids1 = np.array(ids_list).reshape(-1, 1)
            corners2.append(coordinates)
            corners1 = corners2
            corners1 = np.array(corners1, dtype=np.float32)
            corners1 = [np.array([corner], dtype=np.float32) for corner in corners1]
            corners1 = np.array(corners1).reshape(-1, 1, 4, 2)

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size_of_aruco_m, cam_mat, dist_mat)
            center_x = int(np.mean(corners[i][0][:, 0]))
            center_y = int(np.mean(corners[i][0][:, 1]))
            center_aruco_list.append((center_x, center_y))
            cv2.circle(image, (center_x, center_y), 5, (255, 182, 193), 2)

            distance=tvec[0][0][2]/1000
            distance_from_rgb_list.append(distance)

            R, _ = cv2.Rodrigues(rvec)
            tvec_list.append(tvec)


            def euler_from_matrix(matrix):
                if matrix[2, 0] < 1:
                    if matrix[2, 0] > -1:
                        theta_x = math.atan2(matrix[2, 1], matrix[2, 2])
                        theta_y = math.asin(-matrix[2, 0])
                        theta_z = math.atan2(matrix[1, 0], matrix[0, 0])
                    else:
                        theta_x = 0
                        theta_y = -math.pi / 2
                        theta_z = -math.atan2(-matrix[1, 2], matrix[1, 1])
                else:
                    theta_x = 0
                    theta_y = math.pi / 2
                    theta_z = math.atan2(-matrix[1, 2], matrix[1, 1])

                angle_x_deg = math.degrees(theta_x)
                angle_y_deg = math.degrees(theta_y)
                angle_z_deg = math.degrees(theta_z)

                return angle_x_deg, angle_y_deg, angle_z_deg

            angles = euler_from_matrix(R)
            angle_x_deg, angle_y_deg, angle_z_deg = angles
            angle_aruco_list.append(angle_z_deg)


            if distance < 0.0015: 
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 1, 2)
    cv2.aruco.drawDetectedMarkers(image, corners1, ids1)
    for center in center_aruco_list:
            cv2.putText(image, 'Center', (center[0], center[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids1, tvec_list

class aruco_tf(Node):

    def __init__(self):
     
        super().__init__('aruco_tf_publisher')                               

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)

        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        image_processing_rate = 2                                                   
        self.bridge = CvBridge()                                                       
        self.tf_buffer = tf2_ros.buffer.Buffer()                                       
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                   
        self.timer = self.create_timer(image_processing_rate, self.process_image) 
        # self.display_timer = self.create_timer(0.1, self.display_detected_arucos)
        
        self.detected_arucos_image = None
        self.cv_image = None                                                            
        self.depth_image = None                                                      

    def display_detected_arucos(self):
        if self.detected_arucos_image is not None:
            cv2.imshow('Detected Aruco Markers', self.detected_arucos_image)
            cv2.waitKey(1) 

    def depthimagecb(self, data):

        bridge = CvBridge()
        try:
            self.depth_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(f"Error converting ROS Image to CV2 Image: {e}")

    def colorimagecb(self, data):

        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            if data.encoding == 'bgr8':
                cv_image = cv2.flip(cv_image, 1)  
            self.cv_image = cv_image
        except CvBridgeError as e:
            print(f"Error converting ROS Image to CV2 Image: {e}")

    def process_image(self):

        if self.cv_image is None or self.cv_image.size == 0:
            print("Invalid image data. Skipping processing.")
            return
        
        sizeCamX = 1280
        sizeCamY = 720                                           #   x = depth * (sizeCamX - cX - centerCamX) / focalX
        centerCamX = 640
        centerCamY = 360
        focalX = 931.1829833984375           
        focalY = 931.1829833984375

        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids1, tvec_list = detect_aruco(self.cv_image)
        for i in range(len(ids1)):
            marker_id = int(ids1[i])
            angle_value = angle_aruco_list[i]
            corrected_angle = (0.788 * angle_value) - ((angle_value ** 2) / 3160)
            r = R.from_euler('zyx', [[0.0, 0.0, corrected_angle]], degrees=True)
            quaternion = r.as_quat()
            
            cX, cY = center_aruco_list[i]


            # Check if the coordinates are valid within the depth image
            if 0 <= cY < self.depth_image.shape[0] and 0 <= cX < self.depth_image.shape[1]:
                depth = self.depth_image[cY, cX] / 1000  # Convert mm to m
            else:
                print("Invalid depth coordinate. Skipping processing for this marker.")
                continue
            
            depth= depth-0.2
            depth1=depth+2
            y = (depth * (sizeCamX - cX - centerCamX) / focalX)
            x = (depth1 * (sizeCamY - cY - centerCamY) / focalY)+0.35
            z = (depth +0.2)/ 2
            print("cx,cy:",cX,cY)
            print("\nx:",x,"\ny:",y,"\nz:",z)

            # Publish transform between camera_link and aruco marker center position
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'camera_link'
            transform.child_frame_id = f'cam_{marker_id}'
            transform.transform.translation.x = float(x)
            transform.transform.translation.y = float(y)
            transform.transform.translation.z = float(z)
            transform.transform.rotation.x = quaternion.item(0)  # Convert to float
            transform.transform.rotation.y = quaternion.item(1)  # Convert to float
            transform.transform.rotation.z = quaternion.item(2)  # Convert to float
            transform.transform.rotation.w = quaternion.item(3)  # Convert to float
            self.br.sendTransform(transform)
            try:
                base_to_camera = self.tf_buffer.lookup_transform('base_link', 'camera_link', rclpy.time.Time())
            except tf2_ros.LookupException as e:
                print(f"Lookup Exception: {e}")
                return

            # Publish TF between object frame and base_link
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'base_link'
            transform.child_frame_id = f'obj_{marker_id}'
            transform.transform.translation.x = float(x) #+ base_to_camera.transform.translation.y)
            transform.transform.translation.y = float(y) #+ base_to_camera.transform.translation.x)
            transform.transform.translation.z = float(z) #+ base_to_camera.transform.translation.z)
            transform.transform.rotation.x = quaternion.item(0)  # Convert to float
            transform.transform.rotation.y = quaternion.item(1)  # Convert to float
            transform.transform.rotation.z = quaternion.item(2)  # Convert to float
            transform.transform.rotation.w = quaternion.item(3)  # Convert to float
            self.br.sendTransform(transform)

            self.detected_arucos_image = self.cv_image.copy()

        
def main():

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process

if __name__ == "__main__": 
    main()