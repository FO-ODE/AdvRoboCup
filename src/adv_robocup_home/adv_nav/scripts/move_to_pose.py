#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveToPose:
    def __init__(self):
        rospy.init_node('move_to_pose', anonymous=True)
        
        # Create move_base action client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Move_base action server connected!")
        
        rospy.loginfo("MoveToPose Node Initialized.")
    
    def move_to_goal(self, x, y, z, qx, qy, qz, qw, frame_id="map"):
        """
        Move the robot to the specified pose
        x, y, z: position coordinates
        qx, qy, qz, qw: orientation quaternion
        frame_id: reference coordinate frame
        """
        goal = MoveBaseGoal()
        
        # Set the target pose
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        
        rospy.loginfo(f"Sending goal: Position({x:.3f}, {y:.3f}, {z:.3f}), "
                      f"Orientation({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
        
        # Send the goal
        self.move_base_client.send_goal(goal)
        
        # Wait for result
        rospy.loginfo("Moving to goal...")
        result = self.move_base_client.wait_for_result()
        
        if result:
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached successfully!")
                return True
            else:
                rospy.logwarn(f"Failed to reach goal. State: {state}")
                return False
        else:
            rospy.logwarn("Action timed out!")
            return False
    
    def move_to_preset_pose(self):
        """Move to a preset pose"""
        # Preset position and orientation
        target_x = 1.55
        target_y = -0.2
        target_z = 0.099
        
        target_qx = 0.000
        target_qy = 0.000
        target_qz = 0.967
        target_qw = -0.254
        
        return self.move_to_goal(
            target_x, target_y, target_z,
            target_qx, target_qy, target_qz, target_qw
        )
    
    def cancel_goal(self):
        """Cancel the current goal"""
        rospy.loginfo("Cancelling current goal...")
        self.move_base_client.cancel_all_goals()

def main():
    try:
        mover = MoveToPose()
        
        # Check startup parameter
        mode = rospy.get_param('~mode', 'preset')  # 'preset', 'custom', 'cancel'
        
        if mode == 'preset':
            # Move to preset position
            success = mover.move_to_preset_pose()
            if success:
                rospy.loginfo("Preset mission completed successfully!")
            else:
                rospy.logwarn("Preset mission failed!")
                
        elif mode == 'custom':
            # Retrieve custom pose parameters
            x = rospy.get_param('~x', 1.289)
            y = rospy.get_param('~y', -0.873)
            z = rospy.get_param('~z', 0.099)
            qx = rospy.get_param('~qx', 0.000)
            qy = rospy.get_param('~qy', 0.000)
            qz = rospy.get_param('~qz', 0.967)
            qw = rospy.get_param('~qw', -0.254)
            frame = rospy.get_param('~frame', 'map')
            
            success = mover.move_to_goal(x, y, z, qx, qy, qz, qw, frame)
            if success:
                rospy.loginfo("Custom mission completed successfully!")
            else:
                rospy.logwarn("Custom mission failed!")
                
        elif mode == 'cancel':
            # Cancel the current goal
            mover.cancel_goal()
            
        else:
            rospy.logwarn(f"Unknown mode: {mode}. Use 'preset', 'custom', or 'cancel'.")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("MoveToPose Node terminated.")
    except KeyboardInterrupt:
        rospy.loginfo("MoveToPose Node interrupted by user.")

if __name__ == '__main__':
    main()
