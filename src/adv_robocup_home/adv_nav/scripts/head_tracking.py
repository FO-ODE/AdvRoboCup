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
        self.task_completed = False
        
        rospy.loginfo("Head Tracker Node Initialized.")
        rospy.loginfo("Head will move once to the first received target and then shutdown")
        rospy.loginfo("Send target to /adv_robocup/waving_person/position")
    
    def target_callback(self, msg):
        """处理目标点回调 - 只移动一次然后关闭节点"""
        if not self.tracking_enabled or self.task_completed:
            return
            
        if self.has_moved:
            return
        
        rospy.loginfo(f"Received target: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}) in {msg.header.frame_id}")
        
        try:
            # 将目标点转换到base_link坐标系
            target_in_head_frame = self.tf_buffer.transform(
                msg, 
                "base_link",
                rospy.Duration(1.0)
            )
            
            target_point = target_in_head_frame.point
            
            # 只计算水平角度 (Pan angle)
            target_pan_angle = math.atan2(target_point.y, target_point.x)
            
            # 应用角度限制
            target_pan_angle = max(-self.pan_limit, min(self.pan_limit, target_pan_angle))
            
            rospy.loginfo(f"Moving head to: pan={math.degrees(target_pan_angle):.1f}°")
            
            # 移动头部
            success = self.move_head(target_pan_angle, self.tilt_angle)
            
            # 标记任务完成
            self.has_moved = True
            self.task_completed = True
            
            if success:
                rospy.loginfo("SUCCESS: Head movement completed!")
                rospy.loginfo("Task completed. Shutting down node...")
                rospy.signal_shutdown("Head tracking completed successfully")
            else:
                rospy.logerr("FAILED: Head movement failed!")
                rospy.logerr("Task failed. Shutting down node...")
                rospy.signal_shutdown("Head tracking failed")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Could not transform target point: {e}")
            rospy.logerr("Task failed. Shutting down node...")
            rospy.signal_shutdown("Coordinate transformation failed")
    
    def move_head(self, pan_angle, tilt_angle):
        """
        移动头部到指定角度
        pan_angle: 水平角度 (左右转动)
        tilt_angle: 垂直角度 (固定为0)
        返回: True if successful, False otherwise
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
        
        # 检查结果
        state = self.head_client.get_state()
        result = self.head_client.get_result()
        
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Head movement action succeeded!")
            return True
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("Head movement action was aborted!")
            return False
        elif state == actionlib.GoalStatus.REJECTED:
            rospy.logwarn("Head movement action was rejected!")
            return False
        else:
            rospy.logwarn(f"Head movement ended with state: {state}")
            return False
    
    def run(self):
        """主循环 - 保持节点运行直到任务完成"""
        rospy.loginfo("Head tracker running. Waiting for target point...")
        rospy.loginfo("Will move once to the first received target and then shutdown")
        
        # 简单的自旋，等待回调
        rospy.spin()

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