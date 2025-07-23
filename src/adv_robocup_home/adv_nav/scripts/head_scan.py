#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import math

class HeadScanner:
    def __init__(self):
        rospy.init_node('head_scanner', anonymous=True)
        
        # Connect to head controller action server
        self.head_client = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for head controller action server...")
        self.head_client.wait_for_server()
        rospy.loginfo("Head controller action server connected!")
        
        # Subscribe to stop/start/center commands
        rospy.Subscriber('/head_scan_command', String, self.command_callback)
        
        # Head joint names (for TIAGo robot)
        self.head_joint_names = ['head_1_joint', 'head_2_joint']
        
        # Scan parameters - slower movement
        self.scan_range = math.radians(60)     # Scan range ±60 degrees
        self.scan_speed = 10                   # Scan speed (seconds per sweep)
        self.pause_time = 1.0                  # Pause time at each position
        
        # Control flag
        self.stop_scanning = False
        
        rospy.loginfo("Head Scanner Node Initialized.")
    
    def command_callback(self, msg):
        """Handle control commands"""
        command = msg.data.lower().strip()
        rospy.loginfo(f"Received command: {command}")
        
        if command == "stop":
            rospy.loginfo("Stop command received! Stopping head scanning...")
            self.stop_scanning = True
            # Cancel the current head action
            self.head_client.cancel_all_goals()
            # Do not automatically return to center; stay at current position
        elif command == "start":
            rospy.loginfo("Start command received! Resuming head scanning...")
            self.stop_scanning = False
        elif command == "center":
            rospy.loginfo("Center command received! Moving head to center...")
            # Only return to center when 'center' command is explicitly sent
            self.return_to_center()
    
    def move_head(self, pan_angle, tilt_angle=0.0, duration=2.5):
        """
        Move head to specified angles
        pan_angle: horizontal angle (yaw)
        tilt_angle: vertical angle (pitch)
        duration: movement duration (seconds) - increased from 1.0 to 2.5
        """
        # Check if scanning should stop
        if self.stop_scanning:
            return None
            
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.head_joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [pan_angle, tilt_angle]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now()
        
        # Send goal
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()
        
        return self.head_client.get_result()
    
    def scan_sequence(self):
        """Execute one complete left-right scan sequence"""
        positions = [
            (-self.scan_range, "left"),
            ( self.scan_range, "right"),
        ]
        
        for pan_angle, name in positions:
            # Check if scanning should stop
            if self.stop_scanning:
                rospy.loginfo("Scan sequence interrupted by stop command. Head stopped at current position.")
                break
                
            rospy.loginfo(f"Moving head to {name} position: {math.degrees(pan_angle):.1f}°")
            
            # Move to position
            result = self.move_head(pan_angle, 0.0, self.scan_speed)
            
            if result:
                rospy.loginfo(f"Head moved to {name}")
            else:
                if self.stop_scanning:
                    rospy.loginfo("Head movement stopped by stop command. Staying at current position.")
                else:
                    rospy.logwarn(f"Failed to move head to {name}")
                break
                
            # Pause at the position (but check for stop signal)
            start = rospy.Time.now()
            while (rospy.Time.now() - start).to_sec() < self.pause_time:
                if self.stop_scanning:
                    rospy.loginfo("Pause interrupted by stop command. Head remains at current position.")
                    return
                rospy.sleep(0.1)
    
    def continuous_scan(self):
        """Continuous scanning mode"""
        rospy.loginfo("Starting continuous head scanning...")
        rospy.loginfo("Send 'stop' to /head_scan_command to stop scanning")
        rospy.loginfo("Send 'center' to /head_scan_command to return to center")
        
        while not rospy.is_shutdown() and not self.stop_scanning:
            try:
                self.scan_sequence()
                if self.stop_scanning:
                    break
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Error during head scanning: {e}")
                break
                
        rospy.loginfo("Continuous scanning stopped. Head remains at current position.")
    
    def single_scan(self):
        """Perform a single scan then stop"""
        rospy.loginfo("Performing single head scan...")
        rospy.loginfo("Send 'stop' to /head_scan_command to interrupt scanning")
        rospy.loginfo("Send 'center' to /head_scan_command to return to center")
        try:
            self.scan_sequence()
        except Exception as e:
            rospy.logerr(f"Error during head scanning: {e}")
        
        if not self.stop_scanning:
            rospy.loginfo("Single scan completed.")
        else:
            rospy.loginfo("Single scan interrupted. Head remains at current position.")
    
    def return_to_center(self):
        """Return head to center position"""
        rospy.loginfo("Returning head to center position...")
        # Temporarily disable stop flag to ensure completion of center return
        prev = self.stop_scanning
        self.stop_scanning = False
        self.move_head(0.0, 0.0, 1.0)  # Quick return
        self.stop_scanning = prev

def main():
    try:
        scanner = HeadScanner()
        
        # Check startup parameter
        mode = rospy.get_param('~mode', 'continuous')  # 'single' or 'continuous'
        scanner.return_to_center()
        
        if mode == 'continuous':
            scanner.continuous_scan()
        elif mode == 'single':
            scanner.single_scan()
        elif mode == 'center':
            scanner.return_to_center()
        else:
            rospy.logwarn(f"Unknown scan mode: {mode}. Using 'single' mode.")
            scanner.single_scan()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Head Scanner Node terminated.")
    except KeyboardInterrupt:
        rospy.loginfo("Head Scanner Node interrupted by user.")

if __name__ == '__main__':
    main()
