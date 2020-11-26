#!/usr/bin/env python
import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus

# Brings in the messages used by the MoveBase action, including the
# goal message and the result message.
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseActionResult

if __name__ == '__main__':
    # Initialize a ROS Node
    rospy.init_node('move_docent')

    # Create a SimpleActionClient for the move_base action server.
    docent_navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait until the move_base action server becomes available.
    rospy.loginfo("Waiting for move_base action server to come up...")
    docent_navigation_client.wait_for_server()
    rospy.loginfo("move_base action server is available...")

    # Creates a goal to send to the action server.
    docent_robot1_goal = MoveBaseGoal()

    # Construct the target pose for the docent in the "map" frame.
    docent_robot1_goal.target_pose.header.stamp = rospy.Time.now()
    docent_robot1_goal.target_pose.header.frame_id = "map"
    docent_robot1_goal.target_pose.header.seq = 1
    docent_robot1_goal.target_pose.pose.position.x = 0
    docent_robot1_goal.target_pose.pose.position.y = 0
    docent_robot1_goal.target_pose.pose.position.z = 0.0
    docent_robot1_goal.target_pose.pose.orientation.x = 0.0
    docent_robot1_goal.target_pose.pose.orientation.y = 0.0
    docent_robot1_goal.target_pose.pose.orientation.z = 0.0
    docent_robot1_goal.target_pose.pose.orientation.w = 1.0

    # Send the goal to the action server.
    try:
        docent_navigation_client.send_goal(docent_robot1_goal)
        rospy.loginfo("Goal sent to move_base action server.")
        rospy.loginfo("Goal position x: %s  y: %s", docent_robot1_goal.target_pose.pose.position.x, docent_robot1_goal.target_pose.pose.position.y)

        # Wait for the server to finish performing the action.
        docent_navigation_client.wait_for_result()

        # Display a log message depending on the navigation result.
        navigation_result_status = docent_navigation_client.get_state()
        if GoalStatus.SUCCEEDED != navigation_result_status:
            rospy.logerr('Navigation to the desired goal failed :( -- Sorry, try again!)')
        else:
            rospy.loginfo('Hooray! Successfully reached the desired goal!')

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt received to stop ROS node.")