#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveToPose:
    def __init__(self):
        rospy.init_node('move_to_pose', anonymous=True)
        
        # 创建move_base action客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Move_base action server connected!")
        
        rospy.loginfo("Move to Pose Node Initialized.")
    
    def move_to_goal(self, x, y, z, qx, qy, qz, qw, frame_id="map"):
        """
        移动机器人到指定位姿
        x, y, z: 位置坐标
        qx, qy, qz, qw: 四元数表示的朝向
        frame_id: 坐标系
        """
        goal = MoveBaseGoal()
        
        # 设置目标位姿
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        
        rospy.loginfo(f"Sending goal: Position({x:.3f}, {y:.3f}, {z:.3f}), Orientation({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
        
        # 发送目标
        self.move_base_client.send_goal(goal)
        
        # 等待结果
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
        """移动到预设位置"""
        # 目标位置和朝向
        target_x = 1.289
        target_y = -0.873
        target_z = 0.099
        
        target_qx = 0.000
        target_qy = 0.000
        target_qz = 0.967
        target_qw = -0.254
        
        return self.move_to_goal(target_x, target_y, target_z, 
                                target_qx, target_qy, target_qz, target_qw)
    
    def cancel_goal(self):
        """取消当前目标"""
        rospy.loginfo("Cancelling current goal...")
        self.move_base_client.cancel_all_goals()

def main():
    try:
        mover = MoveToPose()
        
        # 检查启动参数
        mode = rospy.get_param('~mode', 'preset')  # 'preset', 'custom', 'cancel'
        
        if mode == 'preset':
            # 移动到预设位置
            success = mover.move_to_preset_pose()
            if success:
                rospy.loginfo("Mission completed successfully!")
            else:
                rospy.logwarn("Mission failed!")
                
        elif mode == 'custom':
            # 从参数获取自定义位置
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
            # 取消当前目标
            mover.cancel_goal()
            
        else:
            rospy.logwarn(f"Unknown mode: {mode}. Use 'preset', 'custom', or 'cancel'")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Move to Pose Node terminated.")
    except KeyboardInterrupt:
        rospy.loginfo("Move to Pose Node interrupted by user.")

if __name__ == '__main__':
    main()
