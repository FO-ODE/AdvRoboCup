#!/usr/bin/env python3
import rospy
import subprocess
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerResponse
from pal_interaction_msgs.msg import TtsActionGoal
from std_msgs.msg import String
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

clip_query = "object"  # 默认初始内容

tts_pub = None  # 全局 Publisher

def clip_query_callback(msg):
    global clip_query
    clip_query = msg.data
    rospy.loginfo(f"Received new clip query: {clip_query}")
    
    
def speak(text, lang_id='en_GB', wait_time=3.0):
    """通过 /tts/goal 播放语音"""
    global tts_pub
    if not tts_pub:
        rospy.logwarn("TTS publisher not initialized!")
        return

    tts_goal = TtsActionGoal()
    tts_goal.goal.rawtext.text = text
    tts_goal.goal.rawtext.lang_id = lang_id

    tts_pub.publish(tts_goal)
    rospy.loginfo(f"Speaking: {text}")
    rospy.sleep(wait_time)


def give_hand_callback(req):
    publish_arm_trajectory()

    text = f"Could you please give me the {clip_query}?"
    speak(text, lang_id='en_GB')

    subprocess.call(["rosrun", "pal_gripper_controller_configuration_gazebo", "home_gripper.py"])
    rospy.sleep(5.0)
    control_gripper(open_gripper=False)
    return TriggerResponse(success=True, message="Give hand completed.")


def handover_callback(req):
    rospy.loginfo("Received handover request. Waiting 5 seconds before opening gripper...")
    rospy.sleep(5.0)

    text = f"Here is your {clip_query}."
    speak(text, lang_id='en_GB')

    control_gripper(open_gripper=True)
    
    rospy.sleep(3.0)

   
    tuck_arm()

    return TriggerResponse(success=True, message="Handover completed with speech and tuck.")


def publish_arm_trajectory():
    pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    rospy.sleep(1.0)

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
    
def tuck_arm():
    rospy.loginfo("Connecting to play_motion action server...")
    client = actionlib.SimpleActionClient("play_motion", PlayMotionAction)
    client.wait_for_server()
    rospy.loginfo("Connected to play_motion.")

    # 确保机器人 joint_states 有值
    rospy.wait_for_message("joint_states", JointState)
    rospy.sleep(1.0)

    rospy.loginfo("Sending 'home' motion goal (tuck arm)...")
    goal = PlayMotionGoal()
    goal.motion_name = 'home'
    goal.skip_planning = False

    client.send_goal(goal)
    finished_before_timeout = client.wait_for_result(rospy.Duration(10.0))

    if finished_before_timeout:
        rospy.loginfo("Arm tucked successfully.")
    else:
        rospy.logwarn("Tuck arm motion did not finish in time.")



def control_gripper(open_gripper=True):
    pub = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
    rospy.sleep(1.0)

    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

    point = JointTrajectoryPoint()
    point.positions = [0.045, 0.045] if open_gripper else [0.0, 0.0]
    point.time_from_start = rospy.Duration(1.0)

    traj.points.append(point)

    action = "Opening" if open_gripper else "Closing"
    rospy.loginfo(f"{action} gripper...")
    pub.publish(traj)


def main():
    global tts_pub
    rospy.init_node("give_hand_node")
    rospy.loginfo("Give hand node started.")

    tts_pub = rospy.Publisher('/tts/goal', TtsActionGoal, queue_size=10)
    rospy.sleep(1.0)
    
    rospy.Subscriber("/adv_robocup/sam2clip/clip_query", String, clip_query_callback)

    rospy.Service("/adv_robocup/give_hand", Trigger, give_hand_callback)
    rospy.Service("/adv_robocup/handover", Trigger, handover_callback)

    rospy.spin()


if __name__ == "__main__":
    main()
