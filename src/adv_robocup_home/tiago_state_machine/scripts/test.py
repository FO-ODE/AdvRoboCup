#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import subprocess
import time
import signal
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
import rospy
import smach
import math
import actionlib
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool ,Float32
from sensor_msgs.msg import PointCloud2, JointState

class MoveToPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.move_to_pose_timeout = 30.0  # 超时设置
    
    def execute(self, userdata):
        rospy.loginfo("=== STATE 1: MOVE TO POSE ===")
        
        cmd = [
            'rosrun', 'adv_nav', 'move_to_pose.py',
            '_mode:=custom',
            '_x:=2.845',
            '_y:=1.305',
            '_z:=0.099',
            '_qx:=0.0',
            '_qy:=0.0',
            '_qz:=-0.465',
            '_qw:=0.885',
            '_frame:=map'
        ]

        success = self.run_process(cmd, self.move_to_pose_timeout)
        
        if success:
            rospy.loginfo("Move to pose completed successfully!")
            return 'success'
        else:
            rospy.logwarn("Move to pose failed or timed out.")
            return 'failure'
    
    def run_process(self, cmd, timeout=None):
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate(timeout=timeout)
            return process.returncode == 0
        except subprocess.TimeoutExpired:
            rospy.logwarn("Process timed out")
            return False
        except Exception as e:
            rospy.logerr(f"Error running process: {e}")
            return False


class HeadScan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['person_detected', 'timeout'])
        self.head_scan_timeout = 30.0
        self.person_detected = False
        self.sub = rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_detected_callback)

    
    def execute(self, userdata):
        rospy.loginfo("=== STATE 2: HEAD SCAN ===")
        self.person_detected = False
        cmd = ['rosrun', 'adv_nav', 'head_scan.py', '_mode:=continuous']
        
        # 启动扫描进程
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        start_time = time.time()
        
        while not self.person_detected and not rospy.is_shutdown():
            if time.time() - start_time > self.head_scan_timeout:
                rospy.logwarn("Head scan timed out")
                return 'timeout'
            rospy.sleep(0.5)
        
        if self.person_detected:
            rospy.loginfo("Person detected!")
            return 'person_detected'
        else:
            rospy.logwarn("No person detected!")
            return 'timeout'
    
    def person_detected_callback(self, msg):
        """人员检测回调"""
        if not self.person_detected:
            rospy.loginfo("Person detected!")
            self.person_detected = True


class HeadTracking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.head_tracking_timeout = 5.0
    
    def execute(self, userdata):
        rospy.loginfo("=== STATE 3: HEAD TRACKING ===")
        cmd = ['rosrun', 'adv_nav', 'head_tracking.py']
        
        # 启动头部跟踪
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(self.head_tracking_timeout)
        
        rospy.loginfo("Head tracking completed")
        return 'success'


# class PersonFollowing(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
    
#     def execute(self, userdata):
#         rospy.loginfo("=== STATE 4: PERSON FOLLOWING ===")
#         cmd = ['rosrun', 'adv_nav', 'person_follower.py']
        
#         try:
#             rospy.loginfo("Starting person_follower.py...")
#             process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            
#             # 等待最长时间（比如120秒），避免无限等
#             stdout, stderr = process.communicate(timeout=30)

#             rospy.loginfo(f"[PersonFollower stdout]:\n{stdout.decode()}")
#             rospy.logwarn(f"[PersonFollower stderr]:\n{stderr.decode()}")

#             if process.returncode == 0:
#                 rospy.loginfo("person_follower.py exited successfully.")
#                 return 'success'
#             else:
#                 rospy.logwarn("person_follower.py exited with error.")
#                 return 'failure'

#         except subprocess.TimeoutExpired:
#             rospy.logwarn("person_follower.py timeout — assuming follow completed.")
#             process.terminate()
#             return 'success'

#         except Exception as e:
#             rospy.logerr(f"person_follower.py crashed: {e}")
#             return 'failure'


class PersonFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.person_sub = rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_callback)
        self.person_pose_robot_frame = None
        self.goal_sent = False
        self.task_completed = False
        self.follow_distance = 1.3

    def person_callback(self, msg):
        if self.goal_sent or self.task_completed:
            return
        try:
            self.tf_buffer.can_transform("base_link", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            person_in_base = self.tf_buffer.transform(msg, "base_link", rospy.Duration(1.0))
            self.person_pose_robot_frame = person_in_base
        except Exception as e:
            rospy.logwarn(f"TF transform failed in callback: {e}")
            return

    def execute(self, userdata):
        rospy.loginfo("=== STATE 4: PERSON FOLLOWING (INTEGRATED) ===")
        self.goal_sent = False
        self.task_completed = False
        self.person_pose_robot_frame = None

        if not self.move_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("move_base server not available")
            return 'failure'

        timeout = rospy.Time.now() + rospy.Duration(20.0)
        rate = rospy.Rate(5)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.person_pose_robot_frame:
                try:
                    pose = self.person_pose_robot_frame.point
                    dist = math.hypot(pose.x, pose.y)
                    if dist < 0.05:
                        rospy.logwarn("Person too close or invalid.")
                        return 'failure'

                    ux, uy = pose.x / dist, pose.y / dist
                    tx, ty = pose.x - ux * self.follow_distance, pose.y - uy * self.follow_distance
                    yaw = math.atan2(pose.y, pose.x)
                    quat = quaternion_from_euler(0, 0, yaw)

                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = "base_link"
                    goal_pose.header.stamp = rospy.Time.now()
                    goal_pose.pose.position.x = tx
                    goal_pose.pose.position.y = ty
                    goal_pose.pose.orientation = Quaternion(*quat)

                    self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
                    goal_map = self.tf_buffer.transform(goal_pose, "map", rospy.Duration(1.0))

                    goal = MoveBaseGoal()
                    goal.target_pose = goal_map
                    self.move_client.send_goal(goal)
                    self.goal_sent = True

                    rospy.loginfo("Sent navigation goal to follow person.")
                    finished = self.move_client.wait_for_result(rospy.Duration(25.0))

                    if finished and self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo("Successfully followed person.")
                        return 'success'
                    else:
                        rospy.logwarn("Failed to reach person-following goal.")
                        return 'failure'

                except Exception as e:
                    rospy.logwarn(f"Error computing follow goal: {e}")
                    return 'failure'

            rate.sleep()

        rospy.logwarn("Timeout while waiting for person pose.")
        return 'failure'


class RecordPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], output_keys=['last_pose'])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):
        rospy.loginfo("=== STATE: RECORD POSE (via tf) ===")
        timeout = time.time() + 5.0

        while not rospy.is_shutdown() and time.time() < timeout:
            try:
                # 尝试获取 map → base_link 的变换
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_link", rospy.Time(0), rospy.Duration(1.0))

                # 构造 Pose
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation

                userdata.last_pose = pose
                rospy.loginfo("✅ Recorded pose from tf: position(%.3f, %.3f), orientation(%.3f, %.3f)", 
                              pose.position.x, pose.position.y, 
                              pose.orientation.z, pose.orientation.w)
                return 'success'

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Waiting for tf transform map → base_link...")
                rospy.sleep(0.1)

        rospy.logwarn("❌ Timeout: couldn't get tf transform.")
        return 'failure'


class ReturnToLastPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['last_pose'])
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo("=== STATE: RETURN TO LAST POSE ===")

        # 检查位置是否存在
        if not hasattr(userdata, 'last_pose') or userdata.last_pose is None:
            rospy.logerr("❌ No valid last_pose found in userdata!")
            return 'failure'

        # 等待 move_base 可用
        rospy.loginfo("Waiting for move_base action server...")
        if not self.move_client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("❌ move_base action server not available.")
            return 'failure'

        # 构造导航目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = userdata.last_pose

        # 输出目标信息
        p = goal.target_pose.pose.position
        q = goal.target_pose.pose.orientation
        rospy.loginfo("Sending goal to pose: position(%.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f)",
                      p.x, p.y, q.x, q.y, q.z, q.w)

        self.move_client.send_goal(goal)

        # 等待结果
        rospy.loginfo("Waiting for robot to reach the recorded pose...")
        finished = self.move_client.wait_for_result(rospy.Duration(40.0))

        if not finished:
            rospy.logwarn("⚠️ move_base timed out while returning to last pose.")
            self.move_client.cancel_goal()
            return 'failure'

        # 检查执行状态
        status = self.move_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ Robot successfully returned to recorded pose.")
            return 'success'
        else:
            rospy.logwarn(f"⚠️ move_base returned with status: {status}")
            return 'failure'


class CenterHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo("=== STATE: CENTER HEAD ===")

        cmd = ['rosrun', 'adv_nav', 'head_scan.py', '_mode:=center']

        try:
            # 执行带参数的 rosrun 命令
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
            
            if result.returncode == 0:
                rospy.loginfo("Head centered successfully via head_scan.py")
                return 'success'
            else:
                rospy.logwarn(f"head_scan.py exited with code {result.returncode}")
                return 'failure'
        except subprocess.TimeoutExpired:
            rospy.logwarn("head_scan.py timed out")
            return 'failure'
        except Exception as e:
            rospy.logerr(f"Error running head_scan.py: {e}")
            return 'failure'


# class ModuleCommunication(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])

#         # 发布器，用于发送启动信号
#         self.start_pub = rospy.Publisher('/adv_robocup/start_signal', String, queue_size=1)

#         # 订阅器，用于监听聊天模块的完成信号
#         self.response_received = False
#         self.response_msg = None
#         self.sub = rospy.Subscriber('/adv_robocup/chat_finished', String, self.response_callback)

#         self.timeout = 60.0  # 等待完成响应的最大时间

#         rospy.sleep(0.5)  # 等待 pub/sub 建立连接（建议）

#     def execute(self, userdata):
#         rospy.loginfo("=== STATE 5: MODULE COMMUNICATION ===")

#         # 发布 "start" 消息
#         start_msg = String()
#         start_msg.data = "start"
#         self.start_pub.publish(start_msg)
#         rospy.loginfo("Published 'start' to /adv_robocup/start_signal")

#         # 等待 "done" 响应
#         start_time = time.time()
#         while not self.response_received and not rospy.is_shutdown():
#             if time.time() - start_time > self.timeout:
#                 rospy.logwarn("Timeout waiting for /adv_robocup/chat_finished")
#                 return 'failure'
#             rospy.sleep(0.1)

#         if self.response_received and self.response_msg.strip().lower() == "done":
#             rospy.loginfo("Received 'done' signal from /adv_robocup/chat_finished")
#             return 'success'
#         else:
#             rospy.logwarn(f"Unexpected response: {self.response_msg}")
#             return 'failure'

#     def response_callback(self, msg):
#         """接收聊天完成的回调函数"""
#         rospy.loginfo(f"chat_finished callback triggered with: {msg.data}")
#         self.response_msg = msg.data
#         self.response_received = True

class ModuleCommunication(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

        # 发布器，用于发送启动信号
        self.start_pub = rospy.Publisher('/adv_robocup/start_signal', String, queue_size=1)

        # 订阅器，用于监听聊天模块的完成信号
        self.response_received = False
        self.response_msg = None
        self.timeout = 60.0  # 等待完成响应的最大时间

        # 注册回调
        self.sub = rospy.Subscriber('/adv_robocup/chat_finished', String, self.response_callback)

        rospy.sleep(0.5)  # 等待 pub/sub 建立连接

    def execute(self, userdata):
        rospy.loginfo("=== STATE 5: MODULE COMMUNICATION ===")

        # ✅ 每次执行前重置状态
        self.response_received = False
        self.response_msg = None

        # 发布 "start" 消息
        self.start_pub.publish(String("start"))
        rospy.loginfo("📨 Published 'start' to /adv_robocup/start_signal")

        # 等待响应
        start_time = time.time()
        while not rospy.is_shutdown():
            if self.response_received:
                if self.response_msg and self.response_msg.strip().lower() == "done":
                    rospy.loginfo("✅ Received 'done' signal from /adv_robocup/chat_finished")
                    return 'success'
                else:
                    rospy.logwarn(f"⚠️ Unexpected message: {self.response_msg}")
                    return 'failure'

            if time.time() - start_time > self.timeout:
                rospy.logwarn("⏰ Timeout waiting for /adv_robocup/chat_finished")
                return 'failure'

            rospy.sleep(0.1)

    def response_callback(self, msg):
        rospy.loginfo(f"🔔 chat_finished callback received: {msg.data}")
        self.response_msg = msg.data
        self.response_received = True




        
class MoveToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.timeout = 30.0  # 超时时间

    def execute(self, userdata):
        rospy.loginfo("=== STATE 6: RETURN TO ORIGIN ===")
        
        cmd = [
            'rosrun', 'adv_nav', 'move_to_pose.py',
            '_mode:=preset'  # 假设这是回到起始点的预设模式
        ]
        
        success = self.run_process(cmd, self.timeout)

        if success:
            rospy.loginfo("Returned to origin successfully.")
            return 'success'
        else:
            rospy.logwarn("Failed to return to origin.")
            return 'failure'

    def run_process(self, cmd, timeout=None):
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate(timeout=timeout)
            return process.returncode == 0
        except subprocess.TimeoutExpired:
            rospy.logwarn("Process timed out")
            return False
        except Exception as e:
            rospy.logerr(f"Error running process: {e}")
            return False

class AdjustPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        rospy.sleep(0.5)  # 等待 Publisher 建立连接

    def execute(self, userdata):
        rospy.loginfo("=== STATE 7: ADJUST POSE ===")

        try:
            # Step 1: 低头
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['head_1_joint', 'head_2_joint']

            point = JointTrajectoryPoint()
            point.positions = [0.0, -0.8]  # 向下看
            point.velocities = [0.0, 0.0]
            point.time_from_start = rospy.Duration(1.0)

            traj_msg.points.append(point)
            traj_msg.header.stamp = rospy.Time.now()
            self.head_pub.publish(traj_msg)
            rospy.loginfo("Published head lowering command.")
            rospy.sleep(1.5)  # 给足够时间执行

            # Step 2: 启动 pose_adjuster 节点
            rospy.loginfo("Starting pose_adjuster.py node...")
            cmd = ['rosrun', 'adv_nav', 'adjust_pose_to_object.py']
            process = subprocess.Popen(cmd)
            
            # 你可以根据需要设置 timeout，或等待其自动 shutdown
            process.wait(timeout=60)
            if process.returncode == 0:
                rospy.loginfo("Pose adjuster completed successfully.")
                return 'success'
            else:
                rospy.logwarn("Pose adjuster exited with error.")
                return 'failure'
        except subprocess.TimeoutExpired:
            rospy.logwarn("Pose adjuster timeout.")
            return 'success'
        except Exception as e:
            rospy.logerr(f"AdjustPose failed: {e}")
            return 'failure'

class GiveHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo("=== STATE 8: GIVE HAND ===")
        try:
            rospy.wait_for_service('/adv_robocup/give_hand', timeout=5)
            give_hand_srv = rospy.ServiceProxy('/adv_robocup/give_hand', Trigger)
            response = give_hand_srv()
            if response.success:
                rospy.loginfo("Give hand succeeded: " + response.message)
                return 'success'
            else:
                rospy.logwarn("Give hand failed: " + response.message)
                return 'failure'
        except Exception as e:
            rospy.logerr(f"Give hand service call failed: {e}")
            return 'failure'


class HandOver(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo("=== STATE 9: HANDOVER ===")
        try:
            rospy.wait_for_service('/adv_robocup/handover', timeout=5)
            handover_srv = rospy.ServiceProxy('/adv_robocup/handover', Trigger)
            response = handover_srv()
            if response.success:
                rospy.loginfo("Handover succeeded: " + response.message)
                return 'success'
            else:
                rospy.logwarn("Handover failed: " + response.message)
                return 'failure'
        except Exception as e:
            rospy.logerr(f"Handover service call failed: {e}")
            return 'failure'

class MoveToTable2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.timeout = 30.0  # 超时时间

    def execute(self, userdata):
        rospy.loginfo("=== STATE 6: RETURN TO ORIGIN ===")
        
        cmd = [
            'rosrun', 'adv_nav', 'move_to_pose.py',
            '_mode:=preset'  # 假设这是回到起始点的预设模式
        ]
        
        success = self.run_process(cmd, self.timeout)

        if success:
            rospy.loginfo("Returned to origin successfully.")
            return 'success'
        else:
            rospy.logwarn("Failed to return to origin.")
            return 'failure'

    def run_process(self, cmd, timeout=None):
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate(timeout=timeout)
            return process.returncode == 0
        except subprocess.TimeoutExpired:
            rospy.logwarn("Process timed out")
            return False
        except Exception as e:
            rospy.logerr(f"Error running process: {e}")
            return False

class FindTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failure'])

        self.object_detected = False

        # 订阅目标识别话题
        self.sub = rospy.Subscriber("/adv_robocup/object_position", PointStamped, self.callback)

        # 可选：通知开始寻找
        self.findtarget_pub = rospy.Publisher('find_target', Bool, queue_size=1)
        self.find_target_msg = Bool(data=True)

    def callback(self, msg):
        # 这里只要收到消息就认为识别成功（你也可以检查位置/标签更精细）
        self.object_detected = True

    def execute(self, userdata):
        rospy.loginfo("Waiting for target object...")

        # 触发检测开始
        self.findtarget_pub.publish(self.find_target_msg)

        timeout = rospy.Time.now() + rospy.Duration(20)
        rate = rospy.Rate(10)
        while rospy.Time.now() < timeout:
            if self.object_detected:
                rospy.loginfo("Object found!")
                return 'succeeded'
            rate.sleep()

        rospy.logwarn("No object detected in time.")
        return 'failure'


class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failure', 'succeeded'])
        
        self.hold_object = False          # 抓取完成标志
        self.gripper_succeed = False      # 抓取是否成功

        # 订阅抓取完成状态
        rospy.Subscriber('/pick_done', Bool, self.hold_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)


        # 发布抓取命令
        self.grasp_command_pub = rospy.Publisher('/grasp_command', Bool, queue_size=1)
        self.gripper_succeed_pub = rospy.Publisher('/gripper_succeed', Bool, queue_size=1)
        self.grasp_done_pub = rospy.Publisher('/grasp_done', Bool, queue_size=1)
        
    def hold_callback(self, data):
        self.hold_object = data.data
        rospy.loginfo(f"Pick done status: {data.data}")
        
    def joint_state_callback(self, data):
        # 确保joint_states中有足够的position数据
        if len(data.position) > 8:
            left_gripper_position = data.position[7]
            right_gripper_position = data.position[8]
            gripper_total = left_gripper_position + right_gripper_position
            
            if gripper_total < 0.02:
                self.gripper_succeed = False
            else:
                self.gripper_succeed = True
                
    def execute(self, userdata):
        rospy.loginfo('[Grasp] Executing grasp state...')
        
        
                # ✅ Step 0: 调用一次 rosservice，触发目标生成
        rospy.loginfo('[Grasp] Triggering /adv_robocup/sam2clip/trigger service...')
        try:
            result = subprocess.run(
                ['rosservice', 'call', '/adv_robocup/sam2clip/trigger'],
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            rospy.loginfo("[Grasp] Service call succeeded:\n" + result.stdout.decode())
        except subprocess.CalledProcessError as e:
            rospy.logwarn("[Grasp] Service call failed:\n" + e.stderr.decode())
            return "failure"
        
        # 重置标志位
        self.hold_object = False
        self.gripper_succeed = False

        # 发布抓取命令
        self.grasp_command_pub.publish(Bool(data=True))
        rospy.loginfo("[Grasp] Grasp command sent to pick.cpp")

        # 等待 pick.cpp 接收到抓取命令并开始抓取（延迟确保位置处理完）
        rospy.sleep(1.0)

        # 等待抓取完成
        timeout = rospy.Time.now() + rospy.Duration(60)
        rate = rospy.Rate(10)
        while rospy.Time.now() < timeout:
            if self.hold_object:
                rospy.loginfo("[Grasp] Pick sequence finished.")
                break
            rate.sleep()

        if not self.hold_object:
            rospy.logwarn("[Grasp] Pick process timeout.")
            return "failure"

        # 稍作等待以确认夹爪状态
        rospy.sleep(2.0)

        if self.gripper_succeed:
            self.gripper_succeed_pub.publish(Bool(data=True))
            self.grasp_done_pub.publish(Bool(data=True))
            rospy.loginfo("[Grasp] Grasp succeeded.")
            return "succeeded"
        else:
            self.gripper_succeed_pub.publish(Bool(data=False))
            self.grasp_done_pub.publish(Bool(data=False))
            rospy.logwarn("[Grasp] Grasp failed: fingers did not close properly.")
            rospy.sleep(4.0)  # 给 torso 留时间复位
            return "failure"


def main():
    rospy.init_node('robot_navigation_state_machine')
    
    # 创建状态机
    sm = smach.StateMachine(outcomes=['completed', 'failed'])
    sm.userdata.last_pose = None  
    
    with sm:
        # smach.StateMachine.add('MOVE_TO_POSE', MoveToPose(), transitions={'success': 'HEAD_SCAN', 'failure': 'failed'})
        # smach.StateMachine.add('HEAD_SCAN', HeadScan(), transitions={'person_detected': 'HEAD_TRACKING', 'timeout': 'failed'})
        # smach.StateMachine.add('HEAD_TRACKING', HeadTracking(), transitions={'success': 'PERSON_FOLLOWING'})
        # smach.StateMachine.add('PERSON_FOLLOWING', PersonFollowing(), transitions={'success': 'CENTER_HEAD', 'failure': 'failed'})
        # smach.StateMachine.add('CENTER_HEAD', CenterHead(), transitions={'success': 'RecordPose', 'failure': 'failed'})
        # smach.StateMachine.add('RecordPose', RecordPose(), transitions={'success': 'MODULE_COMMUNICATION', 'failure': 'failed'}, remapping={'last_pose': 'last_pose'})
        # smach.StateMachine.add('MODULE_COMMUNICATION', ModuleCommunication(), transitions={'success': 'RETURN_TO_ORIGIN', 'failure': 'failed'})
        # smach.StateMachine.add('RETURN_TO_ORIGIN', MoveToTable(), transitions={'success': 'GIVE_HAND', 'failure': 'failed'})
        # smach.StateMachine.add('ADJUST_POSE', AdjustPose(), transitions={'success': 'GIVE_HAND', 'failure': 'failed'})
        # smach.StateMachine.add('GIVE_HAND', GiveHand(), transitions={'success': 'ReturnToLastPose', 'failure': 'failed'})
        # smach.StateMachine.add('ReturnToLastPose', ReturnToLastPose(), transitions={'success': 'HANDOVER', 'failure': 'failed'}, remapping={'last_pose': 'last_pose'})
        # smach.StateMachine.add('HANDOVER', HandOver(), transitions={'success': 'MoveToTable2', 'failure': 'failed'})
        # smach.StateMachine.add('MoveToTable2', MoveToTable2(), transitions={'success': 'MOVE_TO_POSE', 'failure': 'failed'})
        

        # smach.StateMachine.add('RETURN_TO_ORIGIN', MoveToTable(), transitions={'success': 'ADJUST_POSE', 'failure': 'failed'})
        # smach.StateMachine.add('ADJUST_POSE', AdjustPose(), transitions={'success': 'GRASP', 'failure': 'failed'})

        smach.StateMachine.add('GRASP', Grasp(), transitions={'succeeded':'failed','failure':'failed'})

        # smach.StateMachine.add('GIVE_HAND', GiveHand(), transitions={'success': 'ReturnToLastPose', 'failure': 'failed'})
        # smach.StateMachine.add('ReturnToLastPose', ReturnToLastPose(), transitions={'success': 'HANDOVER', 'failure': 'failed'}, remapping={'last_pose': 'last_pose'})
        # smach.StateMachine.add('HANDOVER', HandOver(), transitions={'success': 'MoveToTable2', 'failure': 'failed'})
        # smach.StateMachine.add('MoveToTable2', MoveToTable2(), transitions={'success': 'MOVE_TO_POSE', 'failure': 'failed'})
        
        
    # 创建并启动状态机ROS处理器 
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # 运行状态机
    outcome = sm.execute()
    
    rospy.loginfo("State machine finished with outcome: " + str(outcome))
    sis.stop()

if __name__ == '__main__':
    main()




# #!/usr/bin/env python3

# import rospy
# import smach
# import smach_ros
# import subprocess
# import time
# import signal
# from geometry_msgs.msg import PointStamped
# from std_msgs.msg import String

# class MoveToPose(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
#         self.move_to_pose_timeout = 30.0  # 超时设置
    
#     def execute(self, userdata):
#         rospy.loginfo("=== STATE 1: MOVE TO POSE ===")
        
#         cmd = [
#             'rosrun', 'adv_nav', 'move_to_pose.py',
#             '_mode:=custom',
#             '_x:=2.845',
#             '_y:=1.305',
#             '_z:=0.099',
#             '_qx:=0.0',
#             '_qy:=0.0',
#             '_qz:=-0.465',
#             '_qw:=0.885',
#             '_frame:=map'
#         ]

#         success = self.run_process(cmd, self.move_to_pose_timeout)
        
#         if success:
#             rospy.loginfo("Move to pose completed successfully!")
#             return 'success'
#         else:
#             rospy.logwarn("Move to pose failed or timed out.")
#             return 'failure'
    
#     def run_process(self, cmd, timeout=None):
#         try:
#             process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#             stdout, stderr = process.communicate(timeout=timeout)
#             return process.returncode == 0
#         except subprocess.TimeoutExpired:
#             rospy.logwarn("Process timed out")
#             return False
#         except Exception as e:
#             rospy.logerr(f"Error running process: {e}")
#             return False


# class HeadScan(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['person_detected', 'timeout'])
#         self.head_scan_timeout = 20.0
#         self.person_detected = False
#         self.sub = rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_detected_callback)

    
#     def execute(self, userdata):
#         rospy.loginfo("=== STATE 2: HEAD SCAN ===")
#         self.person_detected = False
#         cmd = ['rosrun', 'adv_nav', 'head_scan.py', '_mode:=continuous']
        
#         # 启动扫描进程
#         process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#         start_time = time.time()
        
#         while not self.person_detected and not rospy.is_shutdown():
#             if time.time() - start_time > self.head_scan_timeout:
#                 rospy.logwarn("Head scan timed out")
#                 return 'timeout'
#             rospy.sleep(0.5)
        
#         if self.person_detected:
#             rospy.loginfo("Person detected!")
#             return 'person_detected'
#         else:
#             rospy.logwarn("No person detected!")
#             return 'timeout'
    
#     def person_detected_callback(self, msg):
#         """人员检测回调"""
#         if not self.person_detected:
#             rospy.loginfo("Person detected!")
#             self.person_detected = True


# class HeadTracking(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success'])
#         self.head_tracking_timeout = 5.0
    
#     def execute(self, userdata):
#         rospy.loginfo("=== STATE 3: HEAD TRACKING ===")
#         cmd = ['rosrun', 'adv_nav', 'head_tracking.py']
        
#         # 启动头部跟踪
#         process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
#         time.sleep(self.head_tracking_timeout)
        
#         rospy.loginfo("Head tracking completed")
#         return 'success'


# class PersonFollowing(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['success'])
    
#     def execute(self, userdata):
#         rospy.loginfo("=== STATE 4: PERSON FOLLOWING ===")
#         cmd = ['rosrun', 'adv_nav', 'person_follower.py']
        
#         rospy.loginfo("Starting person following...")
        
#         process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
#         # 让人员跟随持续运行，直到节点停止
#         process.wait()
        
#         return 'success'


# def main():
#     rospy.init_node('robot_navigation_state_machine')
    
#     # 创建状态机
#     sm = smach.StateMachine(outcomes=['completed', 'failed'])
    
#     with sm:
#         smach.StateMachine.add('MOVE_TO_POSE', MoveToPose(), transitions={'success': 'HEAD_SCAN', 'failure': 'failed'})
#         smach.StateMachine.add('HEAD_SCAN', HeadScan(), transitions={'person_detected': 'HEAD_TRACKING', 'timeout': 'failed'})
#         smach.StateMachine.add('HEAD_TRACKING', HeadTracking(), transitions={'success': 'PERSON_FOLLOWING'})
#         smach.StateMachine.add('PERSON_FOLLOWING', PersonFollowing(), transitions={'success': 'completed'})
    
#     # 创建并启动状态机ROS处理器
#     sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
#     sis.start()
    
#     # 运行状态机
#     outcome = sm.execute()
    
#     rospy.loginfo("State machine finished with outcome: " + str(outcome))
#     sis.stop()

# if __name__ == '__main__':
#     main()





# #!/usr/bin/env python3

# import rospy
# import subprocess
# import threading
# import time
# import signal
# import sys
# from geometry_msgs.msg import PointStamped
# from std_msgs.msg import String

# class RobotNavigationStateMachine:
#     def __init__(self):
#         rospy.init_node('robot_navigation_state_machine', anonymous=True)
        
#         # 状态定义
#         self.STATES = {
#             'MOVE_TO_POSE': 0,
#             'HEAD_SCAN': 1,
#             'HEAD_TRACKING': 2,
#             'PERSON_FOLLOWING': 3,
#             'COMPLETED': 4
#         }
        
#         self.current_state = self.STATES['MOVE_TO_POSE']
#         self.current_process = None
#         self.person_detected = False
        
#         # 订阅人员检测结果
#         rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_detected_callback)
        
#         # 发布头部扫描命令
#         self.head_scan_cmd_pub = rospy.Publisher('/head_scan_command', String, queue_size=1)
        
#         # 状态机参数
#         self.move_to_pose_timeout = 30.0  # 移动到位置的超时时间
#         self.head_scan_timeout = 20.0     # 头部扫描的超时时间
#         self.head_tracking_timeout = 5.0  # 头部跟踪的超时时间
        
#         rospy.loginfo("Robot Navigation State Machine Initialized")
#         rospy.loginfo("States: MOVE_TO_POSE -> HEAD_SCAN -> HEAD_TRACKING -> PERSON_FOLLOWING")
    
#     def person_detected_callback(self, msg):
#         """人员检测回调"""
#         if not self.person_detected:
#             rospy.loginfo("Person detected! Preparing for state transition...")
#             self.person_detected = True
    
#     def run_process(self, cmd, timeout=None):
#         """运行子进程并等待完成或超时"""
#         rospy.loginfo(f"Running command: {' '.join(cmd)}")
        
#         try:
#             self.current_process = subprocess.Popen(
#                 cmd,
#                 stdout=subprocess.PIPE,
#                 stderr=subprocess.PIPE,
#                 preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
#             )
            
#             if timeout:
#                 rospy.loginfo(f"Waiting for process completion (timeout: {timeout}s)...")
#                 try:
#                     stdout, stderr = self.current_process.communicate(timeout=timeout)
#                     return self.current_process.returncode == 0
#                 except subprocess.TimeoutExpired:
#                     rospy.logwarn(f"Process timed out after {timeout}s")
#                     self.terminate_current_process()
#                     return False
#             else:
#                 # 无超时等待
#                 self.current_process.wait()
#                 return self.current_process.returncode == 0
                
#         except Exception as e:
#             rospy.logerr(f"Error running process: {e}")
#             return False
    
#     def terminate_current_process(self):
#         """终止当前运行的进程"""
#         if self.current_process and self.current_process.poll() is None:
#             rospy.loginfo("Terminating current process...")
#             try:
#                 self.current_process.terminate()
#                 time.sleep(1)
#                 if self.current_process.poll() is None:
#                     self.current_process.kill()
#                 self.current_process = None
#             except Exception as e:
#                 rospy.logerr(f"Error terminating process: {e}")
    
#     def state_move_to_pose(self):
#         """状态1: 移动到指定位置"""
#         rospy.loginfo("=== STATE 1: MOVE TO POSE ===")
        
#         cmd = [
#             'rosrun', 'adv_nav', 'move_to_pose.py',
#             '_mode:=custom',
#             '_x:=2.845',
#             '_y:=1.305',
#             '_z:=0.099',
#             '_qx:=0.0',
#             '_qy:=0.0',
#             '_qz:=-0.465',
#             '_qw:=0.885',
#             '_frame:=map'
#         ]
        
#         success = self.run_process(cmd, self.move_to_pose_timeout)
        
#         if success:
#             rospy.loginfo("Move to pose completed successfully!")
#             self.current_state = self.STATES['HEAD_SCAN']
#         else:
#             rospy.logwarn("Move to pose failed or timed out, proceeding anyway...")
#             self.current_state = self.STATES['HEAD_SCAN']
        
#         time.sleep(1)  # 短暂延迟
    
#     def state_head_scan(self):
#         """状态2: 头部扫描寻找人员"""
#         rospy.loginfo("=== STATE 2: HEAD SCAN ===")
        
#         # 启动头部扫描进程
#         cmd = [
#             'rosrun', 'adv_nav', 'head_scan.py',
#             '_mode:=continuous'
#         ]
        
#         rospy.loginfo("Starting head scan to look for person...")
#         self.person_detected = False
        
#         try:
#             self.current_process = subprocess.Popen(
#                 cmd,
#                 stdout=subprocess.PIPE,
#                 stderr=subprocess.PIPE,
#                 preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
#             )
            
#             # 等待检测到人员或超时
#             start_time = time.time()
#             while not self.person_detected and not rospy.is_shutdown():
#                 if time.time() - start_time > self.head_scan_timeout:
#                     rospy.logwarn("Head scan timed out without detecting person")
#                     break
                    
#                 rospy.sleep(0.5)
            
#             # 停止头部扫描
#             if self.person_detected:
#                 rospy.loginfo("Person detected! Stopping head scan...")
#             else:
#                 rospy.loginfo("Head scan timeout reached, stopping scan...")
            
#             # 发送停止命令
#             stop_msg = String()
#             stop_msg.data = "stop"
#             self.head_scan_cmd_pub.publish(stop_msg)
#             time.sleep(0.5)
            
#             # 终止头部扫描进程
#             self.terminate_current_process()
            
#             if self.person_detected:
#                 self.current_state = self.STATES['HEAD_TRACKING']
#             else:
#                 rospy.logwarn("No person detected, transitioning to completed state")
#                 self.current_state = self.STATES['COMPLETED']
            
#         except Exception as e:
#             rospy.logerr(f"Error in head scan state: {e}")
#             self.terminate_current_process()
#             self.current_state = self.STATES['COMPLETED']
        
#         time.sleep(1)
    
#     def state_head_tracking(self):
#         """状态3: 头部跟踪人员"""
#         rospy.loginfo("=== STATE 3: HEAD TRACKING ===")
        
#         cmd = ['rosrun', 'adv_nav', 'head_tracking.py']
        
#         rospy.loginfo("Starting head tracking...")
        
#         try:
#             self.current_process = subprocess.Popen(
#                 cmd,
#                 stdout=subprocess.PIPE,
#                 stderr=subprocess.PIPE,
#                 preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
#             )
            
#             # 让头部跟踪运行一段时间
#             time.sleep(self.head_tracking_timeout)
            
#             rospy.loginfo("Head tracking completed, proceeding to person following...")
#             self.terminate_current_process()
            
#             self.current_state = self.STATES['PERSON_FOLLOWING']
            
#         except Exception as e:
#             rospy.logerr(f"Error in head tracking state: {e}")
#             self.terminate_current_process()
#             self.current_state = self.STATES['PERSON_FOLLOWING']
        
#         time.sleep(1)
    
#     def state_person_following(self):
#         """状态4: 跟随人员"""
#         rospy.loginfo("=== STATE 4: PERSON FOLLOWING ===")
        
#         cmd = ['rosrun', 'adv_nav', 'person_follower.py']
        
#         rospy.loginfo("Starting person following...")
#         rospy.loginfo("Person following will continue until node is stopped...")
        
#         try:
#             self.current_process = subprocess.Popen(
#                 cmd,
#                 stdout=subprocess.PIPE,
#                 stderr=subprocess.PIPE,
#                 preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
#             )
            
#             # 让人员跟随持续运行
#             self.current_process.wait()
            
#         except Exception as e:
#             rospy.logerr(f"Error in person following state: {e}")
        
#         self.current_state = self.STATES['COMPLETED']
    
#     def state_completed(self):
#         """状态5: 完成"""
#         rospy.loginfo("=== STATE MACHINE COMPLETED ===")
#         rospy.loginfo("All navigation tasks finished.")
    
#     def run(self):
#         """运行状态机主循环"""
#         rospy.loginfo("Starting Robot Navigation State Machine...")
        
#         try:
#             while not rospy.is_shutdown():
#                 if self.current_state == self.STATES['MOVE_TO_POSE']:
#                     self.state_move_to_pose()
                    
#                 elif self.current_state == self.STATES['HEAD_SCAN']:
#                     self.state_head_scan()
                    
#                 elif self.current_state == self.STATES['HEAD_TRACKING']:
#                     self.state_head_tracking()
                    
#                 elif self.current_state == self.STATES['PERSON_FOLLOWING']:
#                     self.state_person_following()
                    
#                 elif self.current_state == self.STATES['COMPLETED']:
#                     self.state_completed()
#                     break
                    
#                 else:
#                     rospy.logerr(f"Unknown state: {self.current_state}")
#                     break
                    
#         except KeyboardInterrupt:
#             rospy.loginfo("State machine interrupted by user")
#         except Exception as e:
#             rospy.logerr(f"State machine error: {e}")
#         finally:
#             self.cleanup()
    
#     def cleanup(self):
#         """清理资源"""
#         rospy.loginfo("Cleaning up state machine...")
#         self.terminate_current_process()
        
#         # 发送头部回中心命令
#         try:
#             center_msg = String()
#             center_msg.data = "center"
#             self.head_scan_cmd_pub.publish(center_msg)
#         except:
#             pass

# def signal_handler(sig, frame):
#     rospy.loginfo("Received interrupt signal")
#     rospy.signal_shutdown("Interrupt")
#     sys.exit(0)

# def main():
#     # 设置信号处理
#     signal.signal(signal.SIGINT, signal_handler)
    
#     try:
#         state_machine = RobotNavigationStateMachine()
#         state_machine.run()
        
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Robot Navigation State Machine terminated.")
#     except Exception as e:
#         rospy.logerr(f"State machine failed: {e}")

# if __name__ == '__main__':
#     main()