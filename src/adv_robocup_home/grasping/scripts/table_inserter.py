#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import threading
import numpy as np

class TableCollisionPublisher:
    """
    Publishes a collision object representing a table to the MoveIt PlanningScene 
    based on vertices received from a PointCloud2 topic. (from plane_segmentation node)
    """
    
    def __init__(self):
        rospy.init_node("table_inserter", anonymous=True)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.scene = PlanningSceneInterface(synchronous=True)
        rospy.sleep(2.0) # wait for scene initialization

        self.source_frame = "base_footprint"                # frame of point cloud
        self.target_frame = "odom"                          # MoveIt planning frame
        self.table_added  = False
        self.lock         = threading.Lock()
        rospy.Subscriber("/table_vertices",
                         PointCloud2,
                         self.pointcloud_callback,
                         queue_size=1)

        rospy.loginfo("Waiting for table vertices in {}".format(self.source_frame))


    def pointcloud_callback(self, msg):
        with self.lock:
            pts = list(pc2.read_points(msg, skip_nans=True,
                                       field_names=("x","y","z")))
            if len(pts) != 2:
                rospy.logwarn("Expected 2 vertices, got {}".format(len(pts)))
                return

            p1, p2  = np.array(pts[0]), np.array(pts[1])
            min_xyz = np.minimum(p1, p2)
            max_xyz = np.maximum(p1, p2)


            # extend the table obstacle
            padding_x = 0.1
            padding_y = 1.0

            min_xyz[0] -= padding_x
            max_xyz[0] += padding_x
            min_xyz[1] -= padding_y
            max_xyz[1] += padding_y


            # size and center of the obstacle
            size_x, size_y = max_xyz[:2] - min_xyz[:2] - 0.1
            
            bottom_z       = 0.0                           # to ground
            size_z         = max_xyz[2] - bottom_z
            center_x, center_y = (min_xyz[:2] + max_xyz[:2]) / 2.0
            center_z           = bottom_z + size_z / 2.0 + 0.02

            # pose in base_footprint 
            pose_bf = PoseStamped()
            pose_bf.header.frame_id = self.source_frame
            pose_bf.header.stamp    = rospy.Time(0)
            pose_bf.pose.position.x = float(center_x)
            pose_bf.pose.position.y = float(center_y)
            pose_bf.pose.position.z = float(center_z)
            pose_bf.pose.orientation.w = 1.0               # no rotation

            # to odom
            try:
                T = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        self.source_frame,
                        rospy.Time(0), rospy.Duration(0.5))
                pose_odom = tf2_geometry_msgs.do_transform_pose(pose_bf, T)
                pose_odom.header.frame_id = self.target_frame
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF transform failed: {e}")
                return

            # insert the obstacle to MoveIt
            if not self.table_added:
                self.scene.add_box("table", pose_odom,
                                   size=(size_x, size_y, size_z))
                rospy.logwarn(f"Added collision object 'table' in {self.target_frame}: "
                              f"center=({pose_odom.pose.position.x:.3f}, "
                              f"{pose_odom.pose.position.y:.3f}, "
                              f"{pose_odom.pose.position.z:.3f}), "
                              f"size=({size_x:.3f},{size_y:.3f},{size_z:.3f})")
                self.table_added = True

def main():
    TableCollisionPublisher()
    rospy.spin()

if __name__ == "__main__":
    main()
