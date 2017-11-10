#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from ar_track_alvar.msg import AlvarMarkers

# Listener to handle pulling information about the AR tag(s)
def ar_tag_listener():
    rospy.init_node('ar_tag_subscriber', anonymous=True)
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, ar_tag_callback)
    rospy.spin()

if __name__ == '__main__':
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    while not rospy.is_shutdown():
        raw_input('Position the AR tag and hit <Enter> to compute an IK solution: ')

        # Construct the request
        request = GetPositionIKRequest()
