#!/usr/bin/env python3

import rospy
import subprocess
import threading
import time
import signal
import sys
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

class RobotNavigationStateMachine:
    def __init__(self):
        rospy.init_node('robot_navigation_state_machine', anonymous=True)
        
        # 状态定义
        self.STATES = {
            'MOVE_TO_POSE': 0,
            'HEAD_SCAN': 1,
            'HEAD_TRACKING': 2,
            'PERSON_FOLLOWING': 3,
            'COMPLETED': 4
        }
        
        self.current_state = self.STATES['MOVE_TO_POSE']
        self.current_process = None
        self.person_detected = False
        
        # 订阅人员检测结果
        rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_detected_callback)
        
        # 发布头部扫描命令
        self.head_scan_cmd_pub = rospy.Publisher('/head_scan_command', String, queue_size=1)
        
        # 状态机参数
        self.move_to_pose_timeout = 30.0  # 移动到位置的超时时间
        self.head_scan_timeout = 20.0     # 头部扫描的超时时间
        self.head_tracking_timeout = 5.0  # 头部跟踪的超时时间
        
        rospy.loginfo("Robot Navigation State Machine Initialized")
        rospy.loginfo("States: MOVE_TO_POSE -> HEAD_SCAN -> HEAD_TRACKING -> PERSON_FOLLOWING")
    
    def person_detected_callback(self, msg):
        """人员检测回调"""
        if not self.person_detected:
            rospy.loginfo("Person detected! Preparing for state transition...")
            self.person_detected = True
    
    def run_process(self, cmd, timeout=None):
        """运行子进程并等待完成或超时"""
        rospy.loginfo(f"Running command: {' '.join(cmd)}")
        
        try:
            self.current_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )
            
            if timeout:
                rospy.loginfo(f"Waiting for process completion (timeout: {timeout}s)...")
                try:
                    stdout, stderr = self.current_process.communicate(timeout=timeout)
                    return self.current_process.returncode == 0
                except subprocess.TimeoutExpired:
                    rospy.logwarn(f"Process timed out after {timeout}s")
                    self.terminate_current_process()
                    return False
            else:
                # 无超时等待
                self.current_process.wait()
                return self.current_process.returncode == 0
                
        except Exception as e:
            rospy.logerr(f"Error running process: {e}")
            return False
    
    def terminate_current_process(self):
        """终止当前运行的进程"""
        if self.current_process and self.current_process.poll() is None:
            rospy.loginfo("Terminating current process...")
            try:
                self.current_process.terminate()
                time.sleep(1)
                if self.current_process.poll() is None:
                    self.current_process.kill()
                self.current_process = None
            except Exception as e:
                rospy.logerr(f"Error terminating process: {e}")
    
    def state_move_to_pose(self):
        """状态1: 移动到指定位置"""
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
            self.current_state = self.STATES['HEAD_SCAN']
        else:
            rospy.logwarn("Move to pose failed or timed out, proceeding anyway...")
            self.current_state = self.STATES['HEAD_SCAN']
        
        time.sleep(1)  # 短暂延迟
    
    def state_head_scan(self):
        """状态2: 头部扫描寻找人员"""
        rospy.loginfo("=== STATE 2: HEAD SCAN ===")
        
        # 启动头部扫描进程
        cmd = [
            'rosrun', 'adv_nav', 'head_scan.py',
            '_mode:=continuous'
        ]
        
        rospy.loginfo("Starting head scan to look for person...")
        self.person_detected = False
        
        try:
            self.current_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )
            
            # 等待检测到人员或超时
            start_time = time.time()
            while not self.person_detected and not rospy.is_shutdown():
                if time.time() - start_time > self.head_scan_timeout:
                    rospy.logwarn("Head scan timed out without detecting person")
                    break
                    
                rospy.sleep(0.5)
            
            # 停止头部扫描
            if self.person_detected:
                rospy.loginfo("Person detected! Stopping head scan...")
            else:
                rospy.loginfo("Head scan timeout reached, stopping scan...")
            
            # 发送停止命令
            stop_msg = String()
            stop_msg.data = "stop"
            self.head_scan_cmd_pub.publish(stop_msg)
            time.sleep(0.5)
            
            # 终止头部扫描进程
            self.terminate_current_process()
            
            if self.person_detected:
                self.current_state = self.STATES['HEAD_TRACKING']
            else:
                rospy.logwarn("No person detected, transitioning to completed state")
                self.current_state = self.STATES['COMPLETED']
            
        except Exception as e:
            rospy.logerr(f"Error in head scan state: {e}")
            self.terminate_current_process()
            self.current_state = self.STATES['COMPLETED']
        
        time.sleep(1)
    
    def state_head_tracking(self):
        """状态3: 头部跟踪人员"""
        rospy.loginfo("=== STATE 3: HEAD TRACKING ===")
        
        cmd = ['rosrun', 'adv_nav', 'head_tracking.py']
        
        rospy.loginfo("Starting head tracking...")
        
        try:
            self.current_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )
            
            # 让头部跟踪运行一段时间
            time.sleep(self.head_tracking_timeout)
            
            rospy.loginfo("Head tracking completed, proceeding to person following...")
            self.terminate_current_process()
            
            self.current_state = self.STATES['PERSON_FOLLOWING']
            
        except Exception as e:
            rospy.logerr(f"Error in head tracking state: {e}")
            self.terminate_current_process()
            self.current_state = self.STATES['PERSON_FOLLOWING']
        
        time.sleep(1)
    
    def state_person_following(self):
        """状态4: 跟随人员"""
        rospy.loginfo("=== STATE 4: PERSON FOLLOWING ===")
        
        cmd = ['rosrun', 'adv_nav', 'person_follower.py']
        
        rospy.loginfo("Starting person following...")
        rospy.loginfo("Person following will continue until node is stopped...")
        
        try:
            self.current_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
            )
            
            # 让人员跟随持续运行
            self.current_process.wait()
            
        except Exception as e:
            rospy.logerr(f"Error in person following state: {e}")
        
        self.current_state = self.STATES['COMPLETED']
    
    def state_completed(self):
        """状态5: 完成"""
        rospy.loginfo("=== STATE MACHINE COMPLETED ===")
        rospy.loginfo("All navigation tasks finished.")
    
    def run(self):
        """运行状态机主循环"""
        rospy.loginfo("Starting Robot Navigation State Machine...")
        
        try:
            while not rospy.is_shutdown():
                if self.current_state == self.STATES['MOVE_TO_POSE']:
                    self.state_move_to_pose()
                    
                elif self.current_state == self.STATES['HEAD_SCAN']:
                    self.state_head_scan()
                    
                elif self.current_state == self.STATES['HEAD_TRACKING']:
                    self.state_head_tracking()
                    
                elif self.current_state == self.STATES['PERSON_FOLLOWING']:
                    self.state_person_following()
                    
                elif self.current_state == self.STATES['COMPLETED']:
                    self.state_completed()
                    break
                    
                else:
                    rospy.logerr(f"Unknown state: {self.current_state}")
                    break
                    
        except KeyboardInterrupt:
            rospy.loginfo("State machine interrupted by user")
        except Exception as e:
            rospy.logerr(f"State machine error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("Cleaning up state machine...")
        self.terminate_current_process()
        
        # 发送头部回中心命令
        try:
            center_msg = String()
            center_msg.data = "center"
            self.head_scan_cmd_pub.publish(center_msg)
        except:
            pass

def signal_handler(sig, frame):
    rospy.loginfo("Received interrupt signal")
    rospy.signal_shutdown("Interrupt")
    sys.exit(0)

def main():
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        state_machine = RobotNavigationStateMachine()
        state_machine.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot Navigation State Machine terminated.")
    except Exception as e:
        rospy.logerr(f"State machine failed: {e}")

if __name__ == '__main__':
    main()