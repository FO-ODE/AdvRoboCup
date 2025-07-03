#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 使用action client代替simple goal publisher
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_client.wait_for_server()
        rospy.loginfo("Move_base action server connected!")

        # 订阅人相对于机器人的坐标
        rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_callback)

        self.person_pose_robot_frame = None  # 存储人相对于机器人base_link坐标系的位置
        self.goal_sent = False  # 标记是否已经发送过目标
        self.task_completed = False  # 标记任务是否完成

        self.follow_distance = 1.3  # 距离人的目标距离 (米)

        rospy.loginfo("Person Follower Node Initialized.")
        rospy.loginfo("Will navigate once to the person's position and then shutdown.")

    def transform_point_to_base_link(self, point_stamped):
        """将PointStamped从任意坐标系转换到base_link坐标系"""
        try:
            # 检查变换是否可用
            self.tf_buffer.can_transform("base_link", point_stamped.header.frame_id, 
                                       rospy.Time(0), rospy.Duration(1.0))
            
            # 执行坐标变换
            point_in_base_link = self.tf_buffer.transform(point_stamped, "base_link", rospy.Duration(1.0))
            
            rospy.loginfo(f"Point transformed from {point_stamped.header.frame_id} to base_link: "
                         f"({point_in_base_link.point.x:.2f}, {point_in_base_link.point.y:.2f}, {point_in_base_link.point.z:.2f})")
            
            return point_in_base_link
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to transform point from {point_stamped.header.frame_id} to base_link: {e}")
            return None

    def person_callback(self, msg):
        """接收到人的PointStamped消息回调"""
        if self.goal_sent or self.task_completed:
            rospy.loginfo("Goal already sent or task completed. Ignoring additional person position updates.")
            return
        
        rospy.loginfo(f"Received person pose: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}) "
                     f"in frame {msg.header.frame_id}")
        
        # 将人的位置转换到base_link坐标系
        person_point_base_link = self.transform_point_to_base_link(msg)
        
        if person_point_base_link is None:
            rospy.logerr("Failed to transform person position to base_link. Cannot proceed.")
            return
        
        # 存储转换后的位置
        self.person_pose_robot_frame = person_point_base_link
        
        # 立即处理并发送目标
        self.process_and_send_goal()

    def process_and_send_goal(self):
        """处理人的位置并发送导航目标"""
        if self.person_pose_robot_frame is None or self.goal_sent or self.task_completed:
            return

        try:
            # 从转换后的PointStamped消息中获取位置信息（已在base_link坐标系中）
            person_position = self.person_pose_robot_frame.point

            # 距离人的X/Y分量（只考虑水平距离）
            person_dist = (person_position.x**2 + person_position.y**2)**0.5

            if person_dist == 0:
                rospy.logwarn("Person is at robot's origin. Cannot calculate follow goal.")
                return

            # 计算单位向量，从机器人指向人
            unit_vec_x = person_position.x / person_dist
            unit_vec_y = person_position.y / person_dist

            # 目标点：从人当前位置向机器人方向后退follow_distance
            target_x = person_position.x - unit_vec_x * self.follow_distance
            target_y = person_position.y - unit_vec_y * self.follow_distance

            # 计算机器人目标朝向：面向人
            target_yaw = math.atan2(person_position.y, person_position.x)

            rospy.loginfo(f"Person distance: {person_dist:.2f}m")
            rospy.loginfo(f"Calculated target position: ({target_x:.2f}, {target_y:.2f})")
            rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f} degrees")

            # 构建目标 PoseStamped (在 base_link 坐标系中)
            goal_pose_base_link = PoseStamped()
            goal_pose_base_link.header.frame_id = "base_link"
            goal_pose_base_link.header.stamp = rospy.Time.now()
            goal_pose_base_link.pose.position.x = target_x
            goal_pose_base_link.pose.position.y = target_y
            goal_pose_base_link.pose.position.z = 0.0 
            
            quat = quaternion_from_euler(0, 0, target_yaw)
            goal_pose_base_link.pose.orientation = Quaternion(*quat)

            # 将目标点从 'base_link' 转换到 'map' 坐标系
            try:
                # 等待 map 到 base_link 的变换
                self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
                goal_pose_map = self.tf_buffer.transform(goal_pose_base_link, "map", rospy.Duration(1.0))

                # 发送目标并等待完成
                self.move_to_goal(goal_pose_map)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Could not transform goal from base_link to map: {e}")
                self.terminate_with_failure()

        except Exception as e:
            rospy.logerr(f"Error processing person position: {e}")
            self.terminate_with_failure()

    def move_to_goal(self, goal_pose_stamped):
        """发送目标给move_base并等待完成"""
        rospy.loginfo(f"Sending goal to move_base: position=({goal_pose_stamped.pose.position.x:.2f}, "
                     f"{goal_pose_stamped.pose.position.y:.2f}), "
                     f"orientation=[{goal_pose_stamped.pose.orientation.x:.3f}, "
                     f"{goal_pose_stamped.pose.orientation.y:.3f}, "
                     f"{goal_pose_stamped.pose.orientation.z:.3f}, "
                     f"{goal_pose_stamped.pose.orientation.w:.3f}], "
                     f"frame={goal_pose_stamped.header.frame_id}")
        
        # 创建MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose_stamped
        
        # 发送目标
        self.move_client.send_goal(goal)
        self.goal_sent = True
        
        rospy.loginfo("Goal sent. Waiting for robot to reach the target...")
        
        # 等待移动完成
        self.move_client.wait_for_result()
        
        # 检查结果
        result = self.move_client.get_result()
        state = self.move_client.get_state()
        
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("SUCCESS: Robot reached the target position!")
            self.task_completed = True
            rospy.loginfo("Task completed. Shutting down node...")
            rospy.signal_shutdown("Navigation completed successfully")
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("ABORTED: Failed to reach the target position")
            self.terminate_with_failure()
        elif state == actionlib.GoalStatus.REJECTED:
            rospy.logwarn("REJECTED: Goal was rejected by move_base")
            self.terminate_with_failure()
        else:
            rospy.logwarn(f"Movement ended with state: {state}")
            self.terminate_with_failure()

    def terminate_with_failure(self):
        """任务失败时的终止处理"""
        rospy.logerr("Navigation task failed. Shutting down node...")
        self.task_completed = True
        rospy.signal_shutdown("Navigation failed")

    def reset_goal_sent(self):
        """重置目标发送标志（用于调试）"""
        self.goal_sent = False
        self.task_completed = False
        rospy.loginfo("Goal sent flag reset. Ready to send new goal.")

    def run(self):
        """主循环 - 保持节点运行"""
        rospy.loginfo("Person Follower running. Waiting for person position...")
        rospy.loginfo("Accepts PointStamped from any coordinate frame (will transform to base_link)")
        rospy.loginfo("Will navigate once to the person's position and then shutdown")
        
        # 简单的自旋，等待回调
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Person Follower Node terminated.")
    except KeyboardInterrupt:
        rospy.loginfo("Person Follower Node interrupted by user.")
    except Exception as e:
        rospy.logerr(f"Error in Person Follower: {e}")