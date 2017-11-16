#!/usr/bin/env python
import rospy
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotTrajectory 
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from shape_msgs.msg import SolidPrimitive
from tf import TransformerROS, TransformListener
from moveit_commander import MoveGroupCommander
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import JointState

# Global variables we can access from other functions
transformed_message = None
tf_listener = None
counter = 0

# Camera bias
DEPTH_BIAS = 0.1
PLANNING_BIAS = DEPTH_BIAS + 0.05

# Board origin
board_x = 0
board_y = 0
board_z = 0

# Desired joint constraints
TORSO_POSITION = 0
TOLERANCE = 0.75

# Listener to handle pulling information about the AR tag(s)
def ar_tag_listener():
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, transform_ar_tag)
    rospy.Subscriber('/robot/joint_states', JointState, store_joint)

def store_joint(message): 
    global TORSO_POSITION
    TORSO_POSITION = message.position[1]

def transform_ar_tag(message):
    """
    Input: message of type AlvarMarkers
    Transforms the encoded PoseStamped to the base frame
    and stores it in transformed_message
    """
    global transformed_message
    if message.markers == []:
        return

    global counter
    if counter > 1:
        return
    # Create a TransformerROS to transform the AR tag poses we get
    t = TransformerROS(True, rospy.Duration(10.0))

    # Wait for our transform and get it
    tf_listener.waitForTransform('/head_camera', '/base', rospy.Time(), rospy.Duration(5.0))
    (trans,rot) = tf_listener.lookupTransform('/head_camera', '/base', rospy.Time(0))

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

    pose = message.markers[0].pose # Assume one marker for now
    pose.header.frame_id = '/head_camera'
    transformed_message = t.transformPose('/base', pose)
    global board_x, board_y, board_z
    board_x = transformed_message.pose.position.x
    board_y = transformed_message.pose.position.y
    board_z = transformed_message.pose.position.z
    counter += 1

def inverse_kinematics(): 
    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"
    request.ik_request.ik_link_name = "right_gripper"
    request.ik_request.attempts = 50
    request.ik_request.pose_stamped.header.frame_id = "base"

    # Create joint constraints
    #This is joint constraint will need to be set at the group 
    constraints = Constraints()
    joint_constr = JointConstraint()
    joint_constr.joint_name = "right_j0"
    joint_constr.position = TORSO_POSITION
    joint_constr.tolerance_above = TOLERANCE
    joint_constr.tolerance_below = TOLERANCE
    joint_constr.weight = 0.5
    constraints.joint_constraints.append(joint_constr)
 
    # Get the transformed AR Tag (x,y,z) coordinates
    # Only care about the x coordinate of AR tag; tells use
    # how far away wall is
    x_coord = board_x
    y_coord = board_y
    z_coord = board_z

    y_bias = raw_input("Input y coordinate: ")
    z_bias = raw_input("Input z coordinate: ")

    y_coord += float(y_bias)
    z_coord += float(z_bias)

    #Creating Path Planning 
    waypoints = []
    target_pose = Pose()
    target_pose.position.x = float(x_coord - PLANNING_BIAS)
    target_pose.position.y = float(y_coord)
    target_pose.position.z = float(z_coord)
    target_pose.orientation.y = 1.0/2**(1/2.0)
    target_pose.orientation.w = 1.0/2**(1/2.0)
    waypoints.append(target_pose) 

    #Set the desired orientation for the end effector HERE 
    request.ik_request.pose_stamped.pose.position.x = float(x_coord - 0.18)
    request.ik_request.pose_stamped.pose.position.y = float(y_coord)
    request.ik_request.pose_stamped.pose.position.z = float(z_coord)

    #Set the desired orientation for the end effector HERE 
    #request.ik_request.pose_stamped.pose.orientation.x = float(0.00141)
    #request.ik_request.pose_stamped.pose.orientation.y = float(.6631)
    request.ik_request.pose_stamped.pose.orientation.y = 1.0/2**(1/2.0)
    #request.ik_request.pose_stamped.pose.orientation.z = float(-0.0021)
    #request.ik_request.pose_stamped.pose.orientation.w = float(.74847)
    request.ik_request.pose_stamped.pose.orientation.w = 1.0/2**(1/2.0)
    try: 
        #Send the request to the service 
        response = compute_ik(request)
        
        group = MoveGroupCommander("right_arm")

        #Creating a Robot Trajectory for the Path Planning 
        jump_thres = 0.0
        eef_step = 0.01
        path,fraction = group.compute_cartesian_path(waypoints, eef_step, jump_thres)
        print("Path fraction: {}".format(fraction))
        #Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        #Setting the Joint constraint 
        group.set_path_constraints(constraints) 
        #group.go()
        group.execute(path)
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def add_board_object():
    # Some publisher
    scene_diff_publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size=1)
    rospy.sleep(10.0)
    # Create board object
    board = CollisionObject()
    board.header.frame_id = 'base'
    board.id = 'board'

    board_box = SolidPrimitive()
    board_box.type = 1
    # board_box.dimensions = [3.0, 4.0, 0.185]
    board_box.dimensions = [DEPTH_BIAS*2, 4.0, 3.0]

    board.primitives.append(board_box)

    board_pose = Pose()
    board_pose.position.x = transformed_message.pose.position.x
    board_pose.position.y = transformed_message.pose.position.y
    board_pose.position.z = transformed_message.pose.position.z
    # board_pose.orientation.x = transformed_message.pose.orientation.x
    board_pose.orientation.x = 0
    # board_pose.orientation.y = transformed_message.pose.orientation.y
    board_pose.orientation.y = 0
    # board_pose.orientation.z = transformed_message.pose.orientation.z
    board_pose.orientation.z = 0
    # board_pose.orientation.w = transformed_message.pose.orientation.w
    board_pose.orientation.w = 0

    board.primitive_poses.append(board_pose)

    scene = PlanningScene()
    scene.world.collision_objects.append(board)
    scene.is_diff = True
    scene_diff_publisher.publish(scene)


if __name__ == '__main__':
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    global tf_listener
    # Listen for transforms with tf
    tf_listener = TransformListener()

    # Create a subscriber to get AR tag positions
    ar_tag_listener()

    # Create our board scene object
    add_board_object()

    while not rospy.is_shutdown():
        raw_input('Hit <Enter> to ENGAGE AR tag!')
        inverse_kinematics()
