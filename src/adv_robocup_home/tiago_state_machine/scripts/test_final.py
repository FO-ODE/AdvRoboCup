import smach
import smach_ros
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import math
#from minimind_chat_ros.msg import ChatIntent
import time
from std_msgs.msg import String


# class HeadScan(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
#         self.head_pub = rospy.Publisher("/head_controller/command", JointTrajectory, queue_size=10)

#     def execute(self, userdata):
#         rospy.loginfo("[State: HeadScan] Executing head scan...")
#         try:
#             msg = JointTrajectory()
#             msg.joint_names = ['head_1_joint', 'head_2_joint']

#             # 设置扫描点（左右摆头）
#             point1 = JointTrajectoryPoint()
#             point1.positions = [0.5, -0.8]
#             point1.time_from_start = rospy.Duration(2.0)

#             point2 = JointTrajectoryPoint()
#             point2.positions = [-0.5, -0.8]
#             point2.time_from_start = rospy.Duration(4.0)

#             point3 = JointTrajectoryPoint()
#             point3.positions = [0.0, -0.8]
#             point3.time_from_start = rospy.Duration(6.0)

#             msg.points = [point1, point2, point3]

#             # 发送扫描指令
#             rospy.sleep(1.0)
#             self.head_pub.publish(msg)

#             rospy.sleep(6.5)  # 等待扫描动作完成
#             return 'succeeded'
#         except Exception as e:
#             rospy.logwarn(f"[HeadScan] Failed: {e}")
#             return 'aborted'
        
# class HeadScan(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

#         self.head_client = actionlib.SimpleActionClient(
#             '/head_controller/follow_joint_trajectory',
#             FollowJointTrajectoryAction
#         )
#         rospy.loginfo("Waiting for head controller...")
#         self.head_client.wait_for_server()

#     def execute(self, userdata):
#         try:
#             # 定义左右扫视轨迹
#             trajectory = JointTrajectory()
#             trajectory.joint_names = ['head_1_joint', 'head_2_joint']
            
#             point1 = JointTrajectoryPoint()
#             point1.positions = [math.radians(30), -0.8]
#             point1.time_from_start = rospy.Duration(2.0)

#             point2 = JointTrajectoryPoint()
#             point2.positions = [math.radians(-30), -0.8]
#             point2.time_from_start = rospy.Duration(4.0)

#             point3 = JointTrajectoryPoint()
#             point3.positions = [0.0, -0.8]
#             point3.time_from_start = rospy.Duration(6.0)

#             trajectory.points = [point1, point2, point3]

#             goal = FollowJointTrajectoryGoal()
#             goal.trajectory = trajectory
#             goal.trajectory.header.stamp = rospy.Time.now()

#             self.head_client.send_goal(goal)
#             self.head_client.wait_for_result(timeout=rospy.Duration(7.0))
#             rospy.loginfo("Head scan complete.")
#             return 'succeeded'

#         except Exception as e:
#             rospy.logerr(f"Head scan failed: {e}")
#             return 'aborted'


class WaitForHeadScanStop(smach.State):
    def __init__(self, timeout=30.0):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.received_stop = False
        self.timeout = timeout
        rospy.Subscriber('/head_scan_command', String, self.command_callback)

    def command_callback(self, msg):
        if msg.data.strip().lower() == 'stop':
            rospy.loginfo("[WaitForHeadScanStop] Received 'stop' command.")
            self.received_stop = True

    def execute(self, userdata):
        rospy.loginfo("[State: WaitForHeadScanStop] Waiting for 'stop' command...")
        start_time = rospy.Time.now()

        # 等待直到超时或者收到stop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.received_stop:
                return 'succeeded'
            if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                rospy.logwarn("[WaitForHeadScanStop] Timeout reached without receiving 'stop'.")
                return 'aborted'
            rate.sleep()



class FollowPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])

        self.goal_received = False
        self.goal_pose = None

        self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.goal_received = True
        rospy.loginfo("Received goal from RViz or perception.")

    def execute(self, userdata):
        rospy.loginfo("[State] FOLLOW_PERSON: Waiting for goal...")

        # 等待接收目标
        timeout = rospy.Time.now() + rospy.Duration(10.0)  # 最多等10秒
        while not self.goal_received and rospy.Time.now() < timeout:
            rospy.sleep(0.1)

        if not self.goal_received:
            rospy.logwarn("No goal received within timeout.")
            return 'aborted'

        # 构造并发送导航目标
        goal = MoveBaseGoal()
        goal.target_pose = self.goal_pose
        rospy.loginfo("Sending goal to move_base...")
        self.client.send_goal(goal)
        self.client.wait_for_result()

        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Successfully reached goal.")
            return 'succeeded'
        else:
            rospy.logwarn("Failed to reach goal.")
            return 'aborted'


# class ListenAndRespond(smach.State):
#     def __init__(self, model, tokenizer):
#         smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
#         self.model = model
#         self.tokenizer = tokenizer

#     def execute(self, userdata):
#         try:
#             rospy.loginfo("🎤 Listening to person...")
#             text = record_and_transcribe()
#             rospy.loginfo(f"🗣 Person said: {text}")

#             if text:
#                 run_chat(self.model, self.tokenizer, text)
#                 return 'succeeded'
#             else:
#                 rospy.logwarn("No speech detected.")
#                 return 'aborted'

#         except Exception as e:
#             rospy.logerr(f"Error during ListenAndRespond: {e}")
#             return 'aborted'





# class WaitForIntent(smach.State):
#     def __init__(self, timeout=10.0):
#         smach.State.__init__(self,
#                              outcomes=['succeeded', 'aborted', 'timeout'],
#                              output_keys=['action_out', 'object_out', 'location_out'])
#         self.timeout = timeout
#         self.intent_msg = None

#     def callback(self, msg):
#         rospy.loginfo(f"[WaitForIntent] Received intent: action={msg.action}, object={msg.object}, location={msg.location}")
#         self.intent_msg = msg

#     def execute(self, userdata):
#         rospy.loginfo("[WaitForIntent] Waiting for ChatIntent message...")

#         self.intent_msg = None
#         sub = rospy.Subscriber('/chat_intent', ChatIntent, self.callback)

#         start_time = rospy.Time.now()
#         rate = rospy.Rate(10)  # 10 Hz
#         while not rospy.is_shutdown():
#             if self.intent_msg is not None:
#                 userdata.action_out = self.intent_msg.action
#                 userdata.object_out = self.intent_msg.object
#                 userdata.location_out = self.intent_msg.location
#                 return 'succeeded'

#             if (rospy.Time.now() - start_time).to_sec() > self.timeout:
#                 rospy.logwarn("[WaitForIntent] Timeout waiting for intent message.")
#                 return 'timeout'

#             rate.sleep()

# # 假设你已有 MoveToPose 类
# from move_to_pose import MoveToPose

# class ReturnToFixedPose(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
#         self.navigator = MoveToPose()

#         # ⚠️ 这里填入你调试好后的目标点（map坐标系）
#         self.fixed_pose = {
#             'x': 1.15,
#             'y': -0.88,
#             'z': 0.0,
#             'qx': 0.0,
#             'qy': 0.0,
#             'qz': 0.71,
#             'qw': 0.71,
#             'frame_id': 'map'
#         }

#     def execute(self, userdata):
#         rospy.loginfo("[State: ReturnToFixedPose] Navigating to fixed return point...")
#         success = self.navigator.move_to_goal(
#             self.fixed_pose['x'],
#             self.fixed_pose['y'],
#             self.fixed_pose['z'],
#             self.fixed_pose['qx'],
#             self.fixed_pose['qy'],
#             self.fixed_pose['qz'],
#             self.fixed_pose['qw'],
#             self.fixed_pose['frame_id']
#         )
#         return 'succeeded' if success else 'aborted'


def main():
    rospy.init_node('tiago_state_machine')

    sm = smach.StateMachine(outcomes=['DONE', 'FAILED'])
    
    with sm:
        smach.StateMachine.add('HEAD_SCAN', WaitForHeadScanStop(), transitions={'succeeded': 'FollowPerson', 'aborted': 'FAILED'})

        smach.StateMachine.add('FollowPerson', FollowPerson(), transitions={'succeeded': 'DONE','aborted': 'FAILED'})
        
        smach.StateMachine.add('WaitForIntent', FollowPerson(), transitions={'succeeded': 'DONE','aborted': 'FAILED'})
        
        
        
    outcome = sm.execute()
    rospy.loginfo(outcome)
    rospy.spin()

if __name__ == '__main__':
    main()