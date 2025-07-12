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
        
        # Create tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Connect to the head controller action server
        self.head_client = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for head controller action server...")
        self.head_client.wait_for_server()
        rospy.loginfo("Head controller action server connected!")
        
        # Subscribe to target point messages
        rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.target_callback)
        
        # Head joint names (for TIAGo robot)
        self.head_joint_names = ['head_1_joint', 'head_2_joint']
        
        # Tracking parameters
        self.move_duration = 0.4  # Duration for head movement
        self.tracking_enabled = True
        
        # Head angle limits (radians) - only horizontal movement
        self.pan_limit = math.radians(90)   # ±90 degrees
        self.tilt_angle = 0.0               # Fixed vertical angle (0)
        
        # Flags to track movement state
        self.has_moved = False
        self.task_completed = False
        
        rospy.loginfo("Head Tracker Node Initialized.")
        rospy.loginfo("Head will move once to the first received target and then shut down.")
        rospy.loginfo("Send target to /adv_robocup/waving_person/position")
    
    def target_callback(self, msg):
        """Handle target callback - move only once then shut down the node"""
        if not self.tracking_enabled or self.task_completed:
            return
            
        if self.has_moved:
            return
        
        rospy.loginfo(f"Received target: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}) in frame {msg.header.frame_id}")
        
        try:
            # Transform target point to base_link coordinate frame
            target_in_head_frame = self.tf_buffer.transform(
                msg, 
                "base_link",
                rospy.Duration(1.0)
            )
            
            target_point = target_in_head_frame.point
            
            # Calculate only the horizontal angle (pan)
            target_pan_angle = math.atan2(target_point.y, target_point.x)
            
            # Apply angle limits
            target_pan_angle = max(-self.pan_limit, min(self.pan_limit, target_pan_angle))
            
            rospy.loginfo(f"Moving head to pan={math.degrees(target_pan_angle):.1f}°")
            
            # Move the head
            success = self.move_head(target_pan_angle, self.tilt_angle)
            
            # Mark task completion
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
        Move head to the specified angles
        pan_angle: horizontal angle (yaw)
        tilt_angle: vertical angle (pitch)
        Returns True if successful, False otherwise
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.head_joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [pan_angle, tilt_angle]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(self.move_duration)
        
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now()
        
        # Send goal and wait for completion
        self.head_client.send_goal(goal)
        rospy.loginfo("Waiting for head movement to complete...")
        self.head_client.wait_for_result()
        
        # Check the result
        state = self.head_client.get_state()
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
        """Main loop - keep node running until the task completes"""
        rospy.loginfo("Head tracker running. Waiting for target point...")
        rospy.loginfo("Will move once to the first received target and then shut down.")
        
        # Simple spin, waiting for callbacks
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