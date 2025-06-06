#!/usr/bin/env python3
import rospy
import moveit_commander
from moveit_msgs.msg import Grasp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
import copy
from tf.transformations import quaternion_from_euler
import numpy as np
import tf2_ros
import tf2_geometry_msgs

def create_grasp(target_pose, approach_distance=0.1, retreat_distance=0.1):
    grasp = Grasp()
    grasp.id = "grasp1"
    grasp.grasp_pose = copy.deepcopy(target_pose)

    # finger ahead in x, with 90 deg rotation around x axis
    q = quaternion_from_euler(np.deg2rad(90), 0, 0)
    
    grasp.grasp_pose.pose.orientation.x = q[0]
    grasp.grasp_pose.pose.orientation.y = q[1]
    grasp.grasp_pose.pose.orientation.z = q[2]
    grasp.grasp_pose.pose.orientation.w = q[3]
    
    # offset
    #grasp.grasp_pose.pose.position.x -= 0.2
    #grasp.grasp_pose.pose.position.y += 0.3
    #grasp.grasp_pose.pose.position.z += 0.1
    
    
    # approach before grasping
    grasp.pre_grasp_approach.direction.header.frame_id = "base_footprint"
    grasp.pre_grasp_approach.direction.vector.x = 0.3  # approach from front
    grasp.pre_grasp_approach.min_distance = 0.05
    grasp.pre_grasp_approach.desired_distance = approach_distance

    # retreat after grasping
    grasp.post_grasp_retreat.direction.header.frame_id = "base_footprint"
    grasp.post_grasp_retreat.direction.vector.z = 1.0  # retreat upwards
    grasp.post_grasp_retreat.min_distance = 0.05
    grasp.post_grasp_retreat.desired_distance = retreat_distance


    grasp.pre_grasp_posture = JointTrajectory()
    grasp.pre_grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    point = JointTrajectoryPoint()
    point.positions = [0.045, 0.045]  # maximum: 0.045
    point.time_from_start = rospy.Duration(0.5)
    grasp.pre_grasp_posture.points.append(point)

 
    grasp.grasp_posture = JointTrajectory()
    grasp.grasp_posture.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0]  # close
    point.time_from_start = rospy.Duration(0.5)
    grasp.grasp_posture.points.append(point)

    return grasp

def main():
    rospy.init_node("tiago_grasp")
    moveit_commander.roscpp_initialize([])

    arm_torso = moveit_commander.MoveGroupCommander("arm_torso")
    gripper = moveit_commander.MoveGroupCommander("gripper")
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    
    # get target pose
    obj_name = "grasp_target"
    rospy.loginfo("Waiting for object in scene...")
    start = rospy.get_time()
    while obj_name not in scene.get_known_object_names():
        if rospy.get_time() - start > 10:
            rospy.logerr("Target object not found!")
            return
        rospy.sleep(0.5)

    raw_pose = scene.get_object_poses([obj_name])[obj_name]
    rospy.logwarn(f"[Raw Pose of grasp_target] {raw_pose}")

    planning_frame = arm_torso.get_planning_frame()   # == "odom"
    rospy.logwarn("Planning frame: %s", planning_frame)

    object_pose = PoseStamped()
    object_pose.header.frame_id = planning_frame
    object_pose.pose = raw_pose
    
    


    grasp = create_grasp(object_pose)
    for vec in (grasp.pre_grasp_approach, grasp.post_grasp_retreat):
        vec.direction.header.frame_id = planning_frame
        

    rospy.loginfo("Planning grasp...")
    
    ############################################## test 
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    try:
        trans = tf_buffer.lookup_transform("base_footprint",
                                        grasp.grasp_pose.header.frame_id,
                                        rospy.Time(0),
                                        rospy.Duration(2.0))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(grasp.grasp_pose, trans)
        rospy.loginfo(f"[Grasp Pose in {transformed_pose.header.frame_id}] x={transformed_pose.pose.position.x:.3f}, "
                    f"y={transformed_pose.pose.position.y:.3f}, "
                    f"z={transformed_pose.pose.position.z:.3f}")
    except Exception as e:
        rospy.logwarn("TF Transform failed: " + str(e))
    ############################################## test 
    
    # debug
    arm_torso.set_pose_target(grasp.grasp_pose)
    ok = arm_torso.plan()[0]
    rospy.logwarn("IK ok? %s", ok)
    
    result = arm_torso.pick(obj_name, [grasp])
    
    if result:
        rospy.loginfo("Grasp success!")
    else:
        rospy.logwarn("Grasp failed")

if __name__ == "__main__":
    main()
