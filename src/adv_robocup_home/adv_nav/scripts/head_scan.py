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
        
        # 连接到头部控制器的action server
        self.head_client = actionlib.SimpleActionClient(
            '/head_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        
        rospy.loginfo("Waiting for head controller action server...")
        self.head_client.wait_for_server()
        rospy.loginfo("Head controller action server connected!")
        
        # 订阅停止命令
        rospy.Subscriber('/head_scan_command', String, self.command_callback)
        
        # 头部关节名称 (适用于TIAGo机器人)
        self.head_joint_names = ['head_1_joint', 'head_2_joint']
        
        # 扫视参数 - 调慢移动速度
        self.scan_range = math.radians(60)  # 左右扫视角度范围 (60度)
        self.scan_speed = 10 # 扫视速度 (秒每次转动)
        self.pause_time = 0.0  # 在每个位置停留的时间 - 从0.5增加到1.0
        
        # 控制标志
        self.stop_scanning = False
        
        rospy.loginfo("Head Scanner Node Initialized.")
    
    def command_callback(self, msg):
        """处理控制命令"""
        command = msg.data.lower().strip()
        rospy.loginfo(f"Received command: {command}")
        
        if command == "stop":
            rospy.loginfo("Stop command received! Stopping head scanning...")
            self.stop_scanning = True
            # 取消当前的head action
            self.head_client.cancel_all_goals()
            # 不再自动回到中心位置，保持当前位置
        elif command == "start":
            rospy.loginfo("Start command received! Resuming head scanning...")
            self.stop_scanning = False
        elif command == "center":
            rospy.loginfo("Center command received! Moving head to center...")
            # 只有明确发送center命令才会回到中心
            self.return_to_center()
    
    def move_head(self, pan_angle, tilt_angle=0.0, duration=2.5):
        """
        移动头部到指定角度
        pan_angle: 水平角度 (左右摇头)
        tilt_angle: 垂直角度 (上下点头)
        duration: 移动持续时间 - 从1.0增加到2.5
        """
        # 检查是否需要停止
        if self.stop_scanning:
            return None
            
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.head_joint_names
        
        # 创建轨迹点
        point = JointTrajectoryPoint()
        point.positions = [pan_angle, tilt_angle]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now()
        
        # 发送目标
        self.head_client.send_goal(goal)
        self.head_client.wait_for_result()
        
        return self.head_client.get_result()
    
    def scan_sequence(self):
        """执行一次完整的左右扫视序列"""
        positions = [
            (-self.scan_range, "left"),
            (self.scan_range, "right"),
        ]
        
        for pan_angle, position_name in positions:
            # 检查是否需要停止
            if self.stop_scanning:
                rospy.loginfo(f"Scan sequence interrupted due to stop command. Head stopped at current position.")
                break
                
            rospy.loginfo(f"Moving head to {position_name} position: {math.degrees(pan_angle):.1f} degrees")
            
            # 移动到位置
            result = self.move_head(pan_angle, 0.0, self.scan_speed)
            
            if result:
                rospy.loginfo(f"Head moved to {position_name}")
            else:
                if self.stop_scanning:
                    rospy.loginfo("Head movement stopped due to stop command. Staying at current position.")
                else:
                    rospy.logwarn(f"Failed to move head to {position_name}")
                break
                
            # 在位置停留一段时间（但要检查停止信号）
            sleep_start = rospy.Time.now()
            while (rospy.Time.now() - sleep_start).to_sec() < self.pause_time:
                if self.stop_scanning:
                    rospy.loginfo("Pause interrupted due to stop command. Head remains at current position.")
                    return
                rospy.sleep(0.1)  # 小间隔检查
    
    def continuous_scan(self):
        """持续扫视模式"""
        rospy.loginfo("Starting continuous head scanning...")
        rospy.loginfo("Send 'stop' to /head_scan_command topic to stop scanning")
        rospy.loginfo("Send 'center' to /head_scan_command topic to return to center")
        
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
        """执行一次扫视然后停止"""
        rospy.loginfo("Performing single head scan...")
        rospy.loginfo("Send 'stop' to /head_scan_command topic to interrupt scanning")
        rospy.loginfo("Send 'center' to /head_scan_command topic to return to center") 
        try:
            self.scan_sequence()
        except Exception as e:
            rospy.logerr(f"Error during head scanning: {e}")
        
        if not self.stop_scanning:
            rospy.loginfo("Single scan completed.")
        else:
            rospy.loginfo("Single scan interrupted. Head remains at current position.")
    
    def return_to_center(self):
        """将头部归位到中心"""
        rospy.loginfo("Returning head to center position...")
        # 临时禁用停止标志以确保能完成归位
        temp_stop = self.stop_scanning
        self.stop_scanning = False
        self.move_head(0.0, 0.0, 1.0)  # 快速归位
        self.stop_scanning = temp_stop

def main():
    try:
        scanner = HeadScanner()
        
        # 检查启动参数
        scan_mode = rospy.get_param('~mode', 'continuous')  # 'single' 或 'continuous'
        scanner.return_to_center()
        if scan_mode == 'continuous':
            scanner.continuous_scan()
        elif scan_mode == 'single':
            scanner.single_scan()
        elif scan_mode == 'center':
            scanner.return_to_center()
        else:
            rospy.logwarn(f"Unknown scan mode: {scan_mode}. Using 'single' mode.")
            scanner.single_scan()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Head Scanner Node terminated.")
    except KeyboardInterrupt:
        rospy.loginfo("Head Scanner Node interrupted by user.")

if __name__ == '__main__':
    main()
