#!/usr/bin/env python3

import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
import math
from geometry_msgs.msg import PointStamped
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HeadTracker:
    def __init__(self):
        rospy.init_node('head_tracker', anonymous=True)
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 连接到头部控制器的action server
        self.head_client = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for head controller action server...")
        self.head_client.wait_for_server()
        rospy.loginfo("Head controller action server connected!")
        
        # 订阅目标点
        rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.target_callback)
        
        # 头部关节名称 (适用于TIAGo机器人)
        self.head_joint_names = ['head_1_joint', 'head_2_joint']
        
        # 追踪参数
        self.move_duration = 0.4  # 头部移动时间
        self.tracking_enabled = True
        
        # 头部角度限制 (弧度) - 只限制水平移动
        self.pan_limit = math.radians(90)   # 左右±90度
        self.tilt_angle = 0.0  # 垂直角度固定为0度 (水平)
        
        # 记录是否已经移动过
        self.has_moved = False
        
        rospy.loginfo("Head Tracker Node Initialized.")
        rospy.loginfo("Head will move once to the first received target")
        rospy.loginfo("Send new target to /adv_robocup/waving_person/position")
    
    def target_callback(self, msg):
        """处理目标点回调 - 只移动一次"""
        rospy.loginfo("Received target point for head tracking")
        if not self.tracking_enabled:
            rospy.loginfo("Tracking is disabled")
            return
            
        if self.has_moved:
            # rospy.loginfo("Head has already moved to a target. Ignoring new target.")
            # rospy.loginfo("Restart the node to move to a new target.")
            return
        
        rospy.loginfo(f"Received target: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}) in {msg.header.frame_id}")
        
        try:
            # 将目标点转换到head_1_link坐标系
            target_in_head_frame = self.tf_buffer.transform(
                msg, 
                "head_1_link",  # 头部坐标系
                rospy.Duration(1.0)
            )
            
            target_point = target_in_head_frame.point
            
            # 只计算水平角度 (Pan angle)
            target_pan_angle = math.atan2(target_point.y, target_point.x)
            
            # 应用角度限制
            target_pan_angle = max(-self.pan_limit, min(self.pan_limit, target_pan_angle))
            
            rospy.loginfo(f"Moving head to: pan={math.degrees(target_pan_angle):.1f}°")
            
            # 移动头部
            self.move_head(target_pan_angle, self.tilt_angle)
            
            # 标记已经移动过
            self.has_moved = True
            
            rospy.loginfo("Head movement completed. Node will ignore further targets.")
            rospy.loginfo("To move to a new target, restart this node.")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Could not transform target point: {e}")
    
    def move_head(self, pan_angle, tilt_angle):
        """
        移动头部到指定角度
        pan_angle: 水平角度 (左右转动)
        tilt_angle: 垂直角度 (固定为0)
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.head_joint_names
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = [pan_angle, tilt_angle]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(self.move_duration)
        
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now()
        
        # 发送目标并等待完成
        self.head_client.send_goal(goal)
        rospy.loginfo("Waiting for head movement to complete...")
        self.head_client.wait_for_result()
        
        result = self.head_client.get_result()
        if result:
            rospy.loginfo("Head movement successful!")
        else:
            rospy.logwarn("Head movement failed!")
    
    def reset_tracking(self):
        """重置追踪状态，允许再次移动"""
        self.has_moved = False
        rospy.loginfo("Tracking reset. Ready to move to new target.")
    
    def enable_tracking(self):
        """启用追踪"""
        self.tracking_enabled = True
        rospy.loginfo("Head tracking enabled")
    
    def disable_tracking(self):
        """禁用追踪"""
        self.tracking_enabled = False
        rospy.loginfo("Head tracking disabled")
    
    def return_to_center(self):
        """头部回到中心位置"""
        rospy.loginfo("Returning head to center position...")
        self.move_head(0.0, 0.0)
        # 重置移动状态，允许再次移动到目标
        self.has_moved = False
    
    def run(self):
        """主循环 - 保持节点运行"""
        rospy.loginfo("Head tracker running. Waiting for target point...")
        rospy.loginfo("Will move once to the first received target")

        rate = rospy.Rate(0.1)  # 0.1 Hz
        while not rospy.is_shutdown():
            if self.has_moved:
                rospy.loginfo_throttle(10, "Head tracking completed. Restart node for new target.")
            rate.sleep()

def main():
    try:
        tracker = HeadTracker()
        tracker.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Head Tracker Node terminated.")
    except KeyboardInterrupt:
        rospy.loginfo("Head Tracker Node interrupted by user.")
    except Exception as e:
        rospy.logerr(f"Error in Head Tracker: {e}")

if __name__ == '__main__':
    main()