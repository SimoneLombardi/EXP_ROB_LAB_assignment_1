"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers

from std_msgs.msg import Int16


class ArucoNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Declare and read parameters
        self.declare_parameter("marker_size", .0625)
        self.declare_parameter("aruco_dictionary_id", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("camera_frame", None)

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo,
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        
        # before: cv2.aruco.Dictionary_get(dictionary_id)
        # before: cv2.aruco.Dictionary(dictionary_id, 6, 6)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        # before: cv2.aruco.DetectorParameters_create()
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.bridge = CvBridge()
        
        # ---------------------------------parte mia------------------------------------------- #
        self.image_pub = self.create_publisher(Image, 'image_with_circle', 10)  # pubblica le immagini con il cerchio
        self.stop_pub = self.create_publisher(Int16, 'stop_routine', 10)
        self.mk_id_list = list(())
        self.state = 0                                                            
        # ---------------------------------parte mia------------------------------------------- #
    
    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                                             desired_encoding='mono8')
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame is None:
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame
            
            
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,
                                                                self.aruco_dictionary,
                                                                parameters=self.aruco_parameters)
        if marker_ids is not None:

            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                      self.marker_size, self.intrinsic_mat,
                                                                      self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                   self.marker_size, self.intrinsic_mat,
                                                                   self.distortion)
                
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                
                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])
                
            self.poses_pub.publish(pose_array)
            #print("marker id {}".format(marker_ids))
            self.markers_pub.publish(markers)
        
            # ---------------------------------parte mia------------------------------------------- #
            if (len(self.mk_id_list) < 8 and self.state == 0):  # lista lunghezza minore di 8  --> searching
                for i in marker_ids:
                    if i not in self.mk_id_list:
                        self.mk_id_list.append(i)
                
                self.mk_id_list.sort()
                self.get_logger().info("[ARUCO NODE]----marker ids: {}".format(self.mk_id_list))
                
                # pubblico immagine senza cerchio
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8"))
                
                if len(self.mk_id_list) == 8:
                    self.state = 1
            else:                 
                if self.state == 1:
                    if len(self.mk_id_list) != 0:
                        self.get_logger().info("[ARUCO NODE]----locking for marker: {}".format(self.mk_id_list[0]))
                    
                    for i in marker_ids:
                        
                        if self.mk_id_list[0] in i:
                            indices = np.where(marker_ids == self.mk_id_list[0])[0]
                            center = np.mean(corners[indices[0]], axis=1).astype(int)
                            center = tuple(center.flatten())
                            cv2.circle(cv_image, center, 50, (255, 0, 0), 2)  # Red circle, radius=50
                            
                            if(center[0] > 500): # quando il token trovato arriva oltre lo schermo
                                message = f"[ARUCO NODE]----marker {self.mk_id_list[0]} found, removing from list, marker to go: {len(self.mk_id_list)}"
                                self.get_logger().info(message)
                                self.mk_id_list.pop(0)
                            
                            if len(self.mk_id_list) == 0:
                                #stop routine
                                self.state = 2
                                break
                            
                            
                else:
                    self.get_logger().info("[ARUCO NODE]----routine COMPLETE, stopping robot") 
                    msg = Int16()
                    msg.data = 1
                    self.stop_pub.publish(msg)
                      
                        
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8"))
                        
                
                    
                    
def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
