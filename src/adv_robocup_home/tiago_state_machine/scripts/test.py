#!/usr/bin/env python3
import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import PointCloud2, JointState
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool ,Float32


# 定义一个简单的 State 类
class Human(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['order_received'])

    def execute(self, userdata):
        rospy.loginfo("🤖 正在等待顾客下单...")
        rospy.sleep(2)  # 模拟等待
        return 'order_received'
    
class WaitForOrder(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['order_received'])

    def execute(self, userdata):
        rospy.loginfo("🤖 正在等待顾客下单...")
        rospy.sleep(2)  # 模拟等待
        return 'order_received'

class GoToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived'])

    def execute(self, userdata):
        rospy.loginfo("🚶‍♂️ 正在前往指定餐桌...")
        rospy.sleep(2)
        return 'arrived'

class PickAndDeliver(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("🤖 正在识别并递送物品...")
        rospy.sleep(2)
        return 'done'

def main():
    rospy.init_node('tiago_state_machine')

    # 创建状态机容器
    sm = smach.StateMachine(outcomes=['mission_complete'])

    with sm:
        smach.StateMachine.add('WAIT_FOR_ORDER', WaitForOrder(),
                               transitions={'order_received':'GO_TO_TABLE'})

        smach.StateMachine.add('GO_TO_TABLE', GoToTable(),
                               transitions={'arrived':'PICK_AND_DELIVER'})

        smach.StateMachine.add('PICK_AND_DELIVER', PickAndDeliver(),
                               transitions={'done':'mission_complete'})

    # 执行状态机
    outcome = sm.execute()

if __name__ == '__main__':
    main()
