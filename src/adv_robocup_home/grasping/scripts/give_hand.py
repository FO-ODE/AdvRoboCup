#!/usr/bin/env python3
import rospy
import subprocess
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def publish_arm_trajectory():
    pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    rospy.sleep(1.0)  # 等待发布器建立连接

    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = [
        'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
        'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
    ]

    point = JointTrajectoryPoint()
    point.positions = [0.35, -1.15, -1.0, 2.25, 0.5, 1.1, -2.0]
    point.time_from_start = rospy.Duration(5.0)

    traj.points.append(point)

    rospy.loginfo("Publishing joint trajectory to /arm_controller/command...")
    pub.publish(traj)


def main():
    rospy.init_node("drop_node")
    rospy.loginfo("Drop node started.")

    publish_arm_trajectory()
    rospy.sleep(6.0)  # 等待机械臂运动完成

    rospy.loginfo("Opening gripper...")
    subprocess.call(["rosrun", "pal_gripper_controller_configuration_gazebo", "home_gripper.py"])
    rospy.sleep(1.0)

    rospy.loginfo("Drop task completed.")


if __name__ == "__main__":
    main()
