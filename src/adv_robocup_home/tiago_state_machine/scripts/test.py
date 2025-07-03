#!/usr/bin/env python3

import smach
import smach_ros
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


class WaitForPersonDetection(smach.State):
    """等待人员检测状态 - 订阅检测结果"""
    def __init__(self, timeout=30.0):
        smach.State.__init__(self, outcomes=['person_detected', 'timeout', 'aborted'])
        self.timeout = timeout
        self.person_detected = False
        self.person_position = None
        
        # 发布头部扫描启动命令
        self.scan_cmd_pub = rospy.Publisher('/head_scan_command', String, queue_size=1)
        
        # 订阅人员检测结果
        rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_callback)

    def person_callback(self, msg):
        """人员检测回调"""
        rospy.loginfo(f"[WaitForPersonDetection] Person detected at: ({msg.point.x:.2f}, {msg.point.y:.2f})")
        self.person_detected = True
        self.person_position = msg

    def execute(self, userdata):
        rospy.loginfo("[State: WaitForPersonDetection] Starting person detection...")
        
        # 发送开始扫描命令给head_scan节点
        start_cmd = String()
        start_cmd.data = "start"
        self.scan_cmd_pub.publish(start_cmd)
        rospy.loginfo("Sent 'start' command to head_scan node")
        
        self.person_detected = False
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            # 检查是否检测到人
            if self.person_detected:
                # 停止头部扫描
                stop_cmd = String()
                stop_cmd.data = "stop"
                self.scan_cmd_pub.publish(stop_cmd)
                rospy.loginfo("Person detected! Sent 'stop' command to head_scan node")
                return 'person_detected'
            
            # 检查超时
            if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                rospy.logwarn("[WaitForPersonDetection] Timeout - no person detected")
                # 停止头部扫描
                stop_cmd = String()
                stop_cmd.data = "stop"
                self.scan_cmd_pub.publish(stop_cmd)
                return 'timeout'
            
            rate.sleep()
        
        return 'aborted'


class WaitForHeadTracking(smach.State):
    """等待头部跟踪完成状态"""
    def __init__(self, tracking_duration=10.0):
        smach.State.__init__(self, outcomes=['tracking_complete', 'lost_person', 'aborted'])
        self.tracking_duration = tracking_duration
        self.person_position = None
        self.last_person_time = None
        
        # 订阅人员位置更新
        rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_callback)

    def person_callback(self, msg):
        """人员位置更新回调"""
        self.person_position = msg
        self.last_person_time = rospy.Time.now()

    def execute(self, userdata):
        rospy.loginfo("[State: WaitForHeadTracking] Waiting for head tracking to complete...")
        rospy.loginfo("Note: head_tracking.py node should be running and will track the person automatically")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(5)  # 5Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # 检查跟踪时间
            if (current_time - start_time).to_sec() > self.tracking_duration:
                rospy.loginfo("[WaitForHeadTracking] Head tracking duration completed")
                return 'tracking_complete'
            
            # 检查是否丢失人 (5秒内没有检测更新)
            if (self.last_person_time is not None and 
                (current_time - self.last_person_time).to_sec() > 5.0):
                rospy.logwarn("[WaitForHeadTracking] Lost person during tracking")
                return 'lost_person'
            
            rate.sleep()
        
        return 'aborted'


class WaitForPersonFollowing(smach.State):
    """等待人员跟随完成状态"""
    def __init__(self, following_duration=30.0):
        smach.State.__init__(self, outcomes=['following_complete', 'lost_person', 'aborted'])
        self.following_duration = following_duration
        self.person_position = None
        self.last_person_time = None
        
        # 订阅人员位置更新
        rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_callback)

    def person_callback(self, msg):
        """人员位置更新回调"""
        self.person_position = msg
        self.last_person_time = rospy.Time.now()

    def execute(self, userdata):
        rospy.loginfo("[State: WaitForPersonFollowing] Waiting for person following to complete...")
        rospy.loginfo("Note: person_follower.py node should be running and will follow the person automatically")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(2)  # 2Hz
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # 检查跟随时间
            if (current_time - start_time).to_sec() > self.following_duration:
                rospy.loginfo("[WaitForPersonFollowing] Person following duration completed")
                return 'following_complete'
            
            # 检查是否丢失人 (8秒内没有检测更新)
            if (self.last_person_time is not None and 
                (current_time - self.last_person_time).to_sec() > 8.0):
                rospy.logwarn("[WaitForPersonFollowing] Lost person during following")
                return 'lost_person'
            
            rate.sleep()
        
        return 'aborted'


class SendHeadCenterCommand(smach.State):
    """发送头部回中心命令状态"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        
        # 发布头部控制命令
        self.scan_cmd_pub = rospy.Publisher('/head_scan_command', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("[State: SendHeadCenterCommand] Sending head center command...")
        
        try:
            # 发送center命令给head_scan节点
            center_cmd = String()
            center_cmd.data = "center"
            self.scan_cmd_pub.publish(center_cmd)
            rospy.loginfo("Sent 'center' command to head_scan node")
            
            # 等待一下确保命令被处理
            rospy.sleep(3.0)
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"[SendHeadCenterCommand] Failed: {e}")
            return 'aborted'


def main():
    rospy.init_node('tiago_person_interaction_coordinator')
    
    # 创建顶层状态机
    sm = smach.StateMachine(outcomes=['MISSION_COMPLETE', 'MISSION_FAILED'])
    
    with sm:
        # 添加状态及转换
        smach.StateMachine.add(
            'WAIT_FOR_PERSON_DETECTION', 
            WaitForPersonDetection(timeout=30.0),
            transitions={
                'person_detected': 'WAIT_FOR_HEAD_TRACKING',
                'timeout': 'SEND_HEAD_CENTER_COMMAND',
                'aborted': 'MISSION_FAILED'
            }
        )
        
        smach.StateMachine.add(
            'WAIT_FOR_HEAD_TRACKING', 
            WaitForHeadTracking(tracking_duration=10.0),
            transitions={
                'tracking_complete': 'WAIT_FOR_PERSON_FOLLOWING',
                'lost_person': 'WAIT_FOR_PERSON_DETECTION',
                'aborted': 'MISSION_FAILED'
            }
        )
        
        smach.StateMachine.add(
            'WAIT_FOR_PERSON_FOLLOWING', 
            WaitForPersonFollowing(following_duration=30.0),
            transitions={
                'following_complete': 'SEND_HEAD_CENTER_COMMAND',
                'lost_person': 'WAIT_FOR_PERSON_DETECTION',
                'aborted': 'MISSION_FAILED'
            }
        )
        
        smach.StateMachine.add(
            'SEND_HEAD_CENTER_COMMAND', 
            SendHeadCenterCommand(),
            transitions={
                'succeeded': 'MISSION_COMPLETE',
                'aborted': 'MISSION_FAILED'
            }
        )
    
    # 创建并启动内省服务器（可选，用于可视化）
    sis = smach_ros.IntrospectionServer('tiago_person_interaction', sm, '/SM_ROOT')
    sis.start()
    
    rospy.loginfo("=== TIAGo Person Interaction Coordinator Started ===")
    rospy.loginfo("Required nodes to run separately:")
    rospy.loginfo("  1. head_scan.py")
    rospy.loginfo("  2. head_tracking.py") 
    rospy.loginfo("  3. person_follower.py")
    rospy.loginfo("  4. Person detection node (publishing to /adv_robocup/waving_person/position)")
    rospy.loginfo("")
    rospy.loginfo("State flow: WAIT_FOR_PERSON_DETECTION -> WAIT_FOR_HEAD_TRACKING -> WAIT_FOR_PERSON_FOLLOWING -> SEND_HEAD_CENTER_COMMAND")
    
    # 执行状态机
    outcome = sm.execute()
    
    rospy.loginfo(f"=== State Machine Completed with outcome: {outcome} ===")
    
    # 停止内省服务器
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State machine coordinator interrupted")
    except Exception as e:
        rospy.logerr(f"State machine coordinator error: {e}")