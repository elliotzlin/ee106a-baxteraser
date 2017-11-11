#!/usr/bin/env python
import rospy
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf import TransformerROS, TransformListener
from moveit_commander import MoveGroupCommander
from ar_track_alvar_msgs.msg import AlvarMarkers

# Global variables we can access from other functions
transformed_message = None
transformer = None

# Desired joint constraints
TORSO_POSITION = -1.5
TOLERANCE = 0.5

# Listener to handle pulling information about the AR tag(s)
def ar_tag_listener():
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, transform_ar_tag)

def transform_ar_tag(message):
    """
    Input: message of type AlvarMarkers
    Transforms the encoded PoseStamped to the base frame
    and stores it in transformed_message
    """
    global transformed_message
    if message.markers == []:
        return
    pose = message.markers[0].pose # Assume one marker for now
    pose.header.frame_id = '/head_camera'
    transformed_message = transformer.transformPose('/base', pose)

def inverse_kinematics(message): 
    print(message)
    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_gripper"
    request.ik_request.attempts = 50
    request.ik_request.pose_stamped.header.frame_id = "base"

    # Create joint constraints
    constraints = Constraints()
    joint_constr = JointConstraint()
    joint_constr.joint_name = "right_j0"
    joint_constr.position = TORSO_POSITION
    joint_constr.tolerance_above = TOLERANCE
    joint_constr.tolerance_below = TOLERANCE

    #Get user input of (x,y,z) coordinates
    #x_coord = raw_input('Enter x coordinate: ')
    x_coord = message.pose.position.x
    #y_coord = raw_input('Enter y coordinate: ')
    y_coord = message.pose.position.y
    #z_coord = raw_input('Enter z coordinate: ')
    Z_coord = message.pose.position.z
    print 'x: {} y: {} z: {}'.format(x_coord, y_coord, z_coord)

    #Set the desired position for the end effector HERE 
    request.ik_request.pose_stamped.pose.position.x = float(x_coord)
    request.ik_request.pose_stamped.pose.position.y = float(y_coord)
    request.ik_request.pose_stamped.pose.position.z = float(z_coord)

    # Get quaternions 
    x_quat = message.pose.quaternion.x 
    y_quat = -1.0*message.pose.quaternion.y 
    z_quat = -1.0*message.pose.quaternion.z
    w_quat = message.pose.quaternion.w

    #Set the desired orientation for the end effector HERE 
    request.ik_request.pose_stamped.pose.orientation.x = float(x_quat    )
    request.ik_request.pose_stamped.pose.orientation.y = float(y_quat    )
    request.ik_request.pose_stamped.pose.orientation.z = float(z_quat    )
    request.ik_request.pose_stamped.pose.orientation.w = float(w_quat    )
    try: 
        #Send the request to the service 
        response = compute_ik(request)
        
        #Print the respondse HERE 
        print(response)
        group = MoveGroupCommander("right_arm")

        #Set pose reference frame
        #Comment out the set pose because we are getting the transform with respect to the base 
        #group.set_pose_reference_frame("head_camera")
        #Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        group.go()
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)


    # Listen for transforms with tf
    listener = TransformListener()

    # Create a TransformerROS to transform the AR tag poses we get
    t = TransformerROS(True, rospy.Duration(10.0))

    # Wait for our transform and get it
    listener.waitForTransform('/head_camera', '/base', rospy.Time(), rospy.Duration(5.0))
    (trans,rot) = listener.lookupTransform('/head_camera', '/base', rospy.Time(0))

    # Create our TransformStamped object
    transform = TransformStamped()
    transform.child_frame_id = 'base'
    transform.header.frame_id = 'head_camera'
    transform.header.stamp = rospy.Time(0)
    transform.transform.translation.x = trans[0]
    transform.transform.translation.y = trans[1]
    transform.transform.translation.z = trans[2]
    transform.transform.rotation.x = rot[0]
    transform.transform.rotation.y = rot[1]
    transform.transform.rotation.z = rot[2]
    transform.transform.rotation.w = rot[3]

    # Set the transform for t
    t.setTransform(transform)

    global transformer
    transformer = t

    # Create a subscriber to get AR tag positions
    ar_tag_listener()

    while not rospy.is_shutdown():
        raw_input('Hit <Enter> to ENGAGE AR tag!')
        inverse_kinematics(transformed_message)
