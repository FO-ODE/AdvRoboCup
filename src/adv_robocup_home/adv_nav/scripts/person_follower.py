#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower', anonymous=True)

        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Use action client instead of simple goal publisher
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_client.wait_for_server()
        rospy.loginfo("Move_base action server connected!")

        # Subscribe to the person's position relative to the robot
        rospy.Subscriber('/adv_robocup/waving_person/position', PointStamped, self.person_callback)

        # Store the person's position in the base_link frame
        self.person_pose_robot_frame = None
        # Flag indicating whether a goal has already been sent
        self.goal_sent = False
        # Flag indicating whether the navigation task is completed
        self.task_completed = False

        # Distance to maintain from the person (meters)
        self.follow_distance = 1.3

        rospy.loginfo("Person Follower Node Initialized.")
        rospy.loginfo("Will navigate once to the person's position and then shut down.")

    def transform_point_to_base_link(self, point_stamped):
        """Transform a PointStamped from any frame to the 'base_link' frame"""
        try:
            # Check if the transform is available
            self.tf_buffer.can_transform(
                "base_link",
                point_stamped.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            # Perform the transformation
            point_in_base_link = self.tf_buffer.transform(
                point_stamped,
                "base_link",
                rospy.Duration(1.0)
            )
            rospy.loginfo(
                f"Point transformed from {point_stamped.header.frame_id} to base_link: "
                f"({point_in_base_link.point.x:.2f}, "
                f"{point_in_base_link.point.y:.2f}, "
                f"{point_in_base_link.point.z:.2f})"
            )
            return point_in_base_link
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(
                f"Failed to transform point from {point_stamped.header.frame_id} "
                f"to base_link: {e}"
            )
            return None

    def person_callback(self, msg):
        """Callback for received PointStamped person position"""
        if self.goal_sent or self.task_completed:
            rospy.loginfo(
                "Goal already sent or task completed. Ignoring additional person position updates."
            )
            return

        rospy.loginfo(
            f"Received person pose: "
            f"({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}) "
            f"in frame {msg.header.frame_id}"
        )

        # Transform the person's position to base_link frame
        person_point_base_link = self.transform_point_to_base_link(msg)
        if person_point_base_link is None:
            rospy.logerr("Failed to transform person position to base_link. Cannot proceed.")
            return

        # Store the transformed position
        self.person_pose_robot_frame = person_point_base_link

        # Immediately process and send the navigation goal
        self.process_and_send_goal()

    def process_and_send_goal(self):
        """Process the person's position and send a navigation goal"""
        if (self.person_pose_robot_frame is None or
            self.goal_sent or
            self.task_completed):
            return

        try:
            # Extract the position (already in base_link frame)
            person_position = self.person_pose_robot_frame.point

            # Compute horizontal distance to the person
            person_dist = math.hypot(person_position.x, person_position.y)
            if person_dist == 0:
                rospy.logwarn("Person is at the robot's origin. Cannot calculate follow goal.")
                return

            # Unit vector from robot to person
            unit_vec_x = person_position.x / person_dist
            unit_vec_y = person_position.y / person_dist

            # Target point: step back follow_distance from the person
            target_x = person_position.x - unit_vec_x * self.follow_distance
            target_y = person_position.y - unit_vec_y * self.follow_distance

            # Compute target yaw to face the person
            target_yaw = math.atan2(person_position.y, person_position.x)

            rospy.loginfo(f"Person distance: {person_dist:.2f} m")
            rospy.loginfo(f"Calculated target position: ({target_x:.2f}, {target_y:.2f})")
            rospy.loginfo(f"Target yaw: {math.degrees(target_yaw):.1f} degrees")

            # Build the target PoseStamped in base_link frame
            goal_pose_base_link = PoseStamped()
            goal_pose_base_link.header.frame_id = "base_link"
            goal_pose_base_link.header.stamp = rospy.Time.now()
            goal_pose_base_link.pose.position.x = target_x
            goal_pose_base_link.pose.position.y = target_y
            goal_pose_base_link.pose.position.z = 0.0

            quat = quaternion_from_euler(0, 0, target_yaw)
            goal_pose_base_link.pose.orientation = Quaternion(*quat)

            # Transform the goal to the 'map' frame
            try:
                self.tf_buffer.can_transform(
                    "map", "base_link", rospy.Time(0), rospy.Duration(1.0)
                )
                goal_pose_map = self.tf_buffer.transform(
                    goal_pose_base_link, "map", rospy.Duration(1.0)
                )
                # Send the navigation goal and wait
                self.move_to_goal(goal_pose_map)
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Could not transform goal from base_link to map: {e}")
                self.terminate_with_failure()

        except Exception as e:
            rospy.logerr(f"Error processing person position: {e}")
            self.terminate_with_failure()

    def move_to_goal(self, goal_pose_stamped):
        """Send the goal to move_base and wait for completion"""
        rospy.loginfo(
            f"Sending goal to move_base: "
            f"position=({goal_pose_stamped.pose.position.x:.2f}, "
            f"{goal_pose_stamped.pose.position.y:.2f}), "
            f"orientation=["
            f"{goal_pose_stamped.pose.orientation.x:.3f}, "
            f"{goal_pose_stamped.pose.orientation.y:.3f}, "
            f"{goal_pose_stamped.pose.orientation.z:.3f}, "
            f"{goal_pose_stamped.pose.orientation.w:.3f}], "
            f"frame={goal_pose_stamped.header.frame_id}"
        )

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose_stamped

        self.move_client.send_goal(goal)
        self.goal_sent = True

        rospy.loginfo("Goal sent. Waiting for robot to reach the target...")
        self.move_client.wait_for_result()

        state = self.move_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("SUCCESS: Robot reached the target position!")
            self.task_completed = True
            rospy.loginfo("Task completed. Shutting down node...")
            rospy.signal_shutdown("Navigation completed successfully")
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.logwarn("ABORTED: Failed to reach the target position")
            self.terminate_with_failure()
        elif state == actionlib.GoalStatus.REJECTED:
            rospy.logwarn("REJECTED: Goal was rejected by move_base")
            self.terminate_with_failure()
        else:
            rospy.logwarn(f"Movement ended with state: {state}")
            self.terminate_with_failure()

    def terminate_with_failure(self):
        """Handle task failure and shut down"""
        rospy.logerr("Navigation task failed. Shutting down node...")
        self.task_completed = True
        rospy.signal_shutdown("Navigation failed")

    def reset_goal_sent(self):
        """Reset goal sent flag (for debugging)"""
        self.goal_sent = False
        self.task_completed = False
        rospy.loginfo("Goal sent flag reset. Ready to send new goal.")

    def run(self):
        """Main loop - keep node running until shutdown"""
        rospy.loginfo("Person Follower running. Waiting for person position...")
        rospy.loginfo(
            "Accepts PointStamped from any coordinate frame "
            "(will transform to base_link)"
        )
        rospy.loginfo("Will navigate once to the person's position and then shut down.")

        rospy.spin()

if __name__ == '__main__':
    try:
        follower = PersonFollower()
        follower.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Person Follower Node terminated.")
    except KeyboardInterrupt:
        rospy.loginfo("Person Follower Node interrupted by user.")
    except Exception as e:
        rospy.logerr(f"Error in Person Follower: {e}")