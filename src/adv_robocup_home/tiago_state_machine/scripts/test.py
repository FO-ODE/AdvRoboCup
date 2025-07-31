#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import subprocess
import time
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped
from actionlib_msgs.msg import GoalStatus
import math
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import PointCloud2, JointState


class MoveToPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.move_to_pose_timeout = 50.0  # Timeout setting
    
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
        self.head_scan_timeout = 50.0
        self.person_detected = False
        self.sub = rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_detected_callback)

    def execute(self, userdata):
        rospy.loginfo("=== STATE 2: HEAD SCAN ===")
        self.person_detected = False
        cmd = ['rosrun', 'adv_nav', 'head_scan.py', '_mode:=continuous']
        
        # Start scanning process
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
        """Person detection callback"""
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
        
        # Start head tracking process
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(self.head_tracking_timeout)
        
        rospy.loginfo("Head tracking completed")
        return 'success'



class PersonFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.person_sub = rospy.Subscriber(
            '/adv_robocup/waving_person/position',
            PointStamped,
            self.person_callback
        )

        self.person_pose_robot_frame = None
        self.goal_sent = False
        self.follow_distance = 1.3

    def person_callback(self, msg):
        try:
            if self.tf_buffer.can_transform("base_link", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0)):
                person_in_base = self.tf_buffer.transform(msg, "base_link", rospy.Duration(1.0))
                self.person_pose_robot_frame = person_in_base
            else:
                rospy.logwarn("TF not available to transform person pose to base_link")
        except Exception as e:
            rospy.logwarn(f"TF transform failed in callback: {e}")

    def execute(self, userdata):
        rospy.loginfo("=== STATE 4: PERSON FOLLOWING ===")

        self.goal_sent = False
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
                    if dist < 0.1:
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

                    if not self.tf_buffer.can_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0)):
                        rospy.logwarn("Cannot transform goal to map frame.")
                        rate.sleep()
                        continue

                    goal_map = self.tf_buffer.transform(goal_pose, "map", rospy.Duration(1.0))

                    goal = MoveBaseGoal()
                    goal.target_pose = goal_map
                    self.move_client.send_goal(goal)
                    self.goal_sent = True

                    rospy.loginfo("Sent follow goal.")
                    finished = self.move_client.wait_for_result(rospy.Duration(25.0))

                    if finished and self.move_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo("Successfully followed person.")
                        return 'success'
                    else:
                        rospy.logwarn("Failed to reach follow goal.")
                        return 'failure'

                except Exception as e:
                    rospy.logwarn(f"Error computing or sending goal: {e}")
                    return 'failure'

            rate.sleep()

        rospy.logwarn("Timeout while waiting for valid person pose.")
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
                # Try to get the transform map → base_link
                transform = self.tf_buffer.lookup_transform(
                    "map", "base_link", rospy.Time(0), rospy.Duration(1.0))

                # Construct Pose
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

        # Check if last_pose exists
        if not hasattr(userdata, 'last_pose') or userdata.last_pose is None:
            rospy.logerr("❌ No valid last_pose found in userdata!")
            return 'failure'

        # Wait for move_base server to be available
        rospy.loginfo("Waiting for move_base action server...")
        if not self.move_client.wait_for_server(rospy.Duration(10)):
            rospy.logerr("❌ move_base action server not available.")
            return 'failure'

        # Construct navigation goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = userdata.last_pose

        # Output target info
        p = goal.target_pose.pose.position
        q = goal.target_pose.pose.orientation
        rospy.loginfo("Sending goal to pose: position(%.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f)",
                      p.x, p.y, q.x, q.y, q.z, q.w)

        self.move_client.send_goal(goal)

        # Wait for result
        rospy.loginfo("Waiting for robot to reach the recorded pose...")
        finished = self.move_client.wait_for_result(rospy.Duration(70.0))

        if not finished:
            rospy.logwarn("move_base timed out while returning to last pose.")
            self.move_client.cancel_goal()
            return 'failure'

        # Check execution status
        status = self.move_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ Robot successfully returned to recorded pose.")
            return 'success'
        else:
            rospy.logwarn(f"move_base returned with status: {status}")
            return 'failure'


class CenterHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo("=== STATE: CENTER HEAD ===")

        cmd = ['rosrun', 'adv_nav', 'head_scan.py', '_mode:=center']

        try:
            # Execute rosrun command with parameters
            result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
            
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




class ModuleCommunication(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

        # Publisher to send the start signal
        self.start_pub = rospy.Publisher('/adv_robocup/start_signal', String, queue_size=1)

        # Subscriber to listen for the completion signal from the chat module
        self.response_received = False
        self.response_msg = None
        self.timeout = 60.0  # Maximum wait time for a completion response

        # Register callback
        self.sub = rospy.Subscriber('/adv_robocup/chat_finished', String, self.response_callback)

        rospy.sleep(0.5)  # Wait for pub/sub connections to establish

    def execute(self, userdata):
        rospy.loginfo("=== STATE 5: MODULE COMMUNICATION ===")

        # ✅ Reset state before execution
        self.response_received = False
        self.response_msg = None

        # Publish "start" message
        self.start_pub.publish(String("start"))
        rospy.loginfo("📨 Published 'start' to /adv_robocup/start_signal")

        # Wait for response
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
        self.timeout = 60.0  # Timeout period

    def execute(self, userdata):
        rospy.loginfo("=== STATE 6: RETURN TO ORIGIN ===")
        
        cmd = [
            'rosrun', 'adv_nav', 'move_to_pose.py',
            '_mode:=preset'  # Assume this is a preset mode for returning to the start point
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



class LowerHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        rospy.sleep(1)  # Wait for connection

    def execute(self, userdata):
        rospy.loginfo("=== STATE: LOWER HEAD ===")
        try:
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['head_1_joint', 'head_2_joint']
            point = JointTrajectoryPoint()
            point.positions = [0.0, -0.7]
            point.velocities = [0.0, 0.0]
            point.time_from_start = rospy.Duration(1.0)

            traj_msg.points.append(point)
            traj_msg.header.stamp = rospy.Time.now()
            self.head_pub.publish(traj_msg)
            rospy.loginfo("Head lowering command published.")
            rospy.sleep(10)  # Wait for head to lower

            return 'success'
        except Exception as e:
            rospy.logerr(f"LowerHead failed: {e}")
            return 'failure'


class AdjustToObjectPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo("=== STATE: ADJUST TO OBJECT POSE ===")
        try:
            rospy.loginfo("Starting adjust_pose_to_object.py...")
            cmd = ['rosrun', 'adv_nav', 'adjust_pose_to_object.py']
            process = subprocess.Popen(cmd)
            process.wait(timeout=60)
            if process.returncode == 0:
                rospy.loginfo("Pose adjuster completed successfully.")
                return 'success'
            else:
                rospy.logwarn("Pose adjuster exited with error.")
                return 'failure'
        except subprocess.TimeoutExpired:
            rospy.logwarn("Pose adjuster timeout — continuing anyway.")
            return 'success'
        except Exception as e:
            rospy.logerr(f"AdjustToObjectPose failed: {e}")
            return 'failure'



class RaiseHead(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        rospy.sleep(1)  # Wait for connection

    def execute(self, userdata):
        rospy.loginfo("=== STATE: RAISE HEAD ===")
        try:
            traj_msg = JointTrajectory()
            traj_msg.joint_names = ['head_1_joint', 'head_2_joint']

            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0]  # Restore default posture (looking straight ahead)
            point.velocities = [0.0, 0.0]
            point.time_from_start = rospy.Duration(1.0)

            traj_msg.points.append(point)
            traj_msg.header.stamp = rospy.Time.now()
            self.head_pub.publish(traj_msg)
            rospy.loginfo("Head raise command published.")
            rospy.sleep(6)  # Give enough time to complete the action

            return 'success'
        except Exception as e:
            rospy.logerr(f"RaiseHead failed: {e}")
            return 'failure'


class GiveHand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])

    def execute(self, userdata):
        rospy.loginfo("=== STATE 8: GIVE HAND ===")
        try:
            rospy.wait_for_service('/adv_robocup/give_hand', timeout=20)
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
            rospy.wait_for_service('/adv_robocup/handover', timeout=20)
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
        self.timeout = 70.0  # Timeout period

    def execute(self, userdata):
        rospy.loginfo("=== STATE 6: RETURN TO ORIGIN ===")
        
        cmd = [
            'rosrun', 'adv_nav', 'move_to_pose.py',
            '_mode:=preset'  # Assume this is a preset mode for returning to the starting point
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




class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failure', 'succeeded'])

        self.hold_object = False
        self.gripper_succeed = False

        # Subscribe to grasp completion signal (published by pick.cpp after grasping completes)
        rospy.Subscriber('/pick_done', Bool, self.hold_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.gripper_succeed_pub = rospy.Publisher('/gripper_succeed', Bool, queue_size=1)
        self.grasp_done_pub = rospy.Publisher('/grasp_done', Bool, queue_size=1)

    def hold_callback(self, data):
        self.hold_object = data.data

    def joint_state_callback(self, data):
        if len(data.position) > 8:
            total = data.position[7] + data.position[8]
            self.gripper_succeed = total >= 0.02

    def execute(self, userdata):
        rospy.loginfo('[Grasp] Executing grasp state...')

        # Step 1: Call Trigger service to generate target position
        rospy.loginfo('[Grasp] Calling trigger service to generate target...')
        try:
            rospy.wait_for_service('/adv_robocup/sam2clip/trigger', timeout=20.0)
            trigger_srv = rospy.ServiceProxy('/adv_robocup/sam2clip/trigger', Trigger)
            resp = trigger_srv()
            if not resp.success:
                rospy.logwarn(f"Trigger failed: {resp.message}")
                return 'failure'
        except Exception as e:
            rospy.logwarn(f"Trigger service call failed: {e}")
            return 'failure'

        rospy.sleep(4.0)  # Wait for object_position to be published (so pick service can receive it)

        # Step 2: Call pick service (the modified service in pick.cpp)
        rospy.loginfo('[Grasp] Calling pick service to execute pick action...')
        try:
            rospy.wait_for_service('/adv_robocup/pick_service', timeout=20.0)
            pick_srv = rospy.ServiceProxy('/adv_robocup/pick_service', Trigger)
            resp = pick_srv()
            if not resp.success:
                rospy.logwarn(f"Pick service failed: {resp.message}")
                return 'failure'
        except Exception as e:
            rospy.logwarn(f"Pick service call failed: {e}")
            return 'failure'

        # Step 3: Wait for grasp completion (signaled by pick.cpp publishing /pick_done)
        rospy.loginfo("[Grasp] Waiting for pick_done...")
        self.hold_object = False
        self.gripper_succeed = False

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

        # Step 4: Check if the grasp was successful (based on gripper state)
        rospy.sleep(2.0)
        if self.gripper_succeed:
            self.gripper_succeed_pub.publish(Bool(data=True))
            self.grasp_done_pub.publish(Bool(data=True))
            return "succeeded"
        else:
            self.gripper_succeed_pub.publish(Bool(data=False))
            self.grasp_done_pub.publish(Bool(data=False))
            rospy.logwarn("[Grasp] Gripper did not close properly.")
            return "failure"


def main():
    rospy.init_node('robot_navigation_state_machine')
    
    sm = smach.StateMachine(outcomes=['completed', 'failed'])
    sm.userdata.last_pose = None  
    
    with sm:
        smach.StateMachine.add('MOVE_TO_POSE', MoveToPose(), transitions={'success': 'HEAD_SCAN', 'failure': 'failed'})
        smach.StateMachine.add('HEAD_SCAN', HeadScan(), transitions={'person_detected': 'HEAD_TRACKING', 'timeout': 'failed'})
        smach.StateMachine.add('HEAD_TRACKING', HeadTracking(), transitions={'success': 'PERSON_FOLLOWING'})
        smach.StateMachine.add('PERSON_FOLLOWING', PersonFollowing(), transitions={'success': 'CENTER_HEAD', 'failure': 'failed'})
        smach.StateMachine.add('CENTER_HEAD', CenterHead(), transitions={'success': 'RecordPose', 'failure': 'failed'})
        smach.StateMachine.add('RecordPose', RecordPose(), transitions={'success': 'MODULE_COMMUNICATION', 'failure': 'failed'}, remapping={'last_pose': 'last_pose'})
        smach.StateMachine.add('MODULE_COMMUNICATION', ModuleCommunication(), transitions={'success': 'RETURN_TO_ORIGIN', 'failure': 'failed'})
        smach.StateMachine.add('RETURN_TO_ORIGIN', MoveToTable(), transitions={'success': 'LOWER_HEAD', 'failure': 'failed'})
        smach.StateMachine.add('LOWER_HEAD', LowerHead(), transitions={'success': 'ADJUST_TO_OBJECT_POSE','failure': 'failed'})
        smach.StateMachine.add('ADJUST_TO_OBJECT_POSE', AdjustToObjectPose(), transitions={'success': 'GRASP','failure': 'failed'})
        smach.StateMachine.add('GRASP', Grasp(), transitions={'succeeded':'ReturnToLastPose','failure':'GIVE_HAND'})
        smach.StateMachine.add('GIVE_HAND', GiveHand(), transitions={'success': 'ReturnToLastPose', 'failure': 'failed'})
        smach.StateMachine.add('ReturnToLastPose', ReturnToLastPose(), transitions={'success': 'RaiseHead', 'failure': 'failed'}, remapping={'last_pose': 'last_pose'})
        smach.StateMachine.add('RaiseHead', RaiseHead(), transitions={'success': 'HANDOVER', 'failure': 'failed'})
        smach.StateMachine.add('HANDOVER', HandOver(), transitions={'success': 'MoveToTable2', 'failure': 'failed'})
        smach.StateMachine.add('MoveToTable2', MoveToTable2(), transitions={'success': 'MOVE_TO_POSE', 'failure': 'failed'})
        
    # Create and start SMACH introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()
    
    rospy.loginfo("State machine finished with outcome: " + str(outcome))
    sis.stop()

if __name__ == '__main__':
    main()




