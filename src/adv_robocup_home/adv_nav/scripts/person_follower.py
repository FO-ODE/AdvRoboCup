import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import math

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 移动目标发布器
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # 订阅人相对于机器人的坐标
        rospy.Subscriber('/person_pose', PointStamped, self.person_callback)

        self.person_pose_robot_frame = None  # 存储人相对于机器人坐标系的位置
        self.robot_moving = False  # 标记机器人是否正在执行目标

        self.follow_distance = 1.7  # 距离人的目标距离 (米)
        self.tolerance = 0.3  # 到达目标点的容忍度 (米)
        self.rate = rospy.Rate(10)  # 10 Hz 循环频率

        rospy.loginfo("Person Follower Node Initialized.")

    def person_callback(self, msg):
        # 接收到人的PointStamped消息 (假设是base_link坐标系)
        self.person_pose_robot_frame = msg
        rospy.loginfo(f"Received person pose: {self.person_pose_robot_frame.point.x}, {self.person_pose_robot_frame.point.y}")

    def send_goal(self, goal_pose_stamped):
        """发送目标给move_base"""
        rospy.loginfo(f"Sending new goal to move_base: {goal_pose_stamped.pose.position.x}, {goal_pose_stamped.pose.position.y}")
        self.goal_pub.publish(goal_pose_stamped)
        self.robot_moving = True

    def check_goal_reached(self, target_pose_stamped):
        """
        检查机器人是否已到达目标点。
        注意：这是一个简化检查。move_base通常有自己的成功/失败回调。
        更健壮的实现会使用Actionlib客户端。
        """
        try:
            # 获取机器人当前在目标 frame 中的位姿
            robot_pose_in_target_frame = self.tf_buffer.lookup_transform(
                target_pose_stamped.header.frame_id,
                "base_link",  # 机器人的frame_id
                rospy.Time(0),  # 最新的可用变换
                rospy.Duration(1.0)  # 变换超时时间
            )

            # 计算机器人当前位置与目标位置的距离
            dx = robot_pose_in_target_frame.transform.translation.x - target_pose_stamped.pose.position.x
            dy = robot_pose_in_target_frame.transform.translation.y - target_pose_stamped.pose.position.y
            distance = (dx**2 + dy**2)**0.5

            if distance < self.tolerance:
                rospy.loginfo(f"Goal reached! Distance: {distance:.2f}m")
                self.robot_moving = False
                return True
            else:
                return False
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5, f"TF lookup failed while checking goal: {e}")
            return False

    def run(self):
        while not rospy.is_shutdown():
            if self.person_pose_robot_frame is None:
                rospy.logwarn_throttle(2, "Waiting for person pose...")
                self.robot_moving = False  # 如果没有人，停止移动
            else:
                # 从PointStamped消息中获取位置信息
                person_position = self.person_pose_robot_frame.point

                # 距离人的X/Y分量
                person_dist = (person_position.x**2 + person_position.y**2)**0.5

                if person_dist == 0:
                    rospy.logwarn_throttle(2, "Person is at robot's origin. Cannot calculate follow goal.")
                    self.robot_moving = False
                else:
                    # 计算单位向量，从机器人指向人
                    unit_vec_x = person_position.x / person_dist
                    unit_vec_y = person_position.y / person_dist

                    # 目标点：从人当前位置向机器人方向后退follow_distance
                    target_point_base_link = person_position
                    target_point_base_link.x = person_position.x - unit_vec_x * self.follow_distance
                    target_point_base_link.y = person_position.y - unit_vec_y * self.follow_distance
                    target_point_base_link.z = 0.0  # 保持在地面

                    # 计算机器人目标朝向：面向人
                    target_yaw = math.atan2(person_position.y, person_position.x)

                    # 构建目标 PoseStamped (在 base_link 坐标系中)
                    goal_pose_base_link = PoseStamped()
                    goal_pose_base_link.header.frame_id = "base_link"
                    goal_pose_base_link.header.stamp = rospy.Time.now()
                    goal_pose_base_link.pose.position = target_point_base_link
                    
                    quat = quaternion_from_euler(0, 0, target_yaw)
                    goal_pose_base_link.pose.orientation = Quaternion(*quat)

                    # 将目标点从 'base_link' 转换到 'map' 坐标系
                    try:
                        # 等待 map 到 base_link 的变换
                        self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
                        goal_pose_map = self.tf_buffer.transform(goal_pose_base_link, "map", rospy.Duration(1.0))

                        # 检查是否需要发送新目标
                        # 如果机器人当前不在移动，或者距离目标太远需要更新目标
                        if not self.robot_moving or not self.check_goal_reached(goal_pose_map):
                            self.send_goal(goal_pose_map)
                        else:
                            rospy.loginfo_throttle(5, "Robot is already close to the person-following target.")

                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        rospy.logwarn_throttle(2, f"Could not transform goal from base_link to map: {e}")
                        self.robot_moving = False
                    except Exception as e:
                        rospy.logerr(f"An unexpected error occurred during goal transformation: {e}")
                        self.robot_moving = False

            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Person Follower Node terminated.")