#!/usr/bin/env python
"""
Starter Script for C106B Grasp Planning Lab
Authors: Chris Correa, Riddhi Bagadiaa, Jay Monga
"""
import numpy as np
import cv2
import argparse
from utils import rotation_from_quaternion, create_transform_matrix, quaternion_from_matrix
import trimesh
from policies import GraspingPolicy

try:
    import rospy
    import tf
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import Image, CameraInfo
    from baxter_interface import gripper as baxter_gripper
    from intera_interface import gripper as sawyer_gripper
    from path_planner import PathPlanner
    ros_enabled = True
except:
    print('Couldn\'t import ROS.  I assume you\'re running this on your laptop')
    ros_enabled = False

def lookup_transform(to_frame, from_frame='base'):
    """
    Returns the AR tag position in world coordinates 

    Parameters
    ----------
    to_frame : string
        examples are: ar_marker_7, nozzle, pawn, ar_marker_3, etc
    from_frame : string
        lets be real, you're probably only going to use 'base'

    Returns
    -------
    :4x4 :obj:`numpy.ndarray` relative pose between frames
    """
    if not ros_enabled:
        print('I am the lookup transform function!  ' \
            + 'You\'re not using ROS, so I\'m returning the Identity Matrix.')
        return np.identity(4)
    listener = tf.TransformListener()
    attempts, max_attempts, rate = 0, 500, rospy.Rate(0.1)
    tag_rot=[]
    print("entering")
    while attempts < max_attempts:
        try:
            t = listener.getLatestCommonTime(from_frame, to_frame)
            tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
            attempts = max_attempts
        except:
            print("exception!")
            rate.sleep()
            attempts += 1
    print("exiting")
    rot = rotation_from_quaternion(tag_rot)
    return create_transform_matrix(rot, tag_pos)


def execute_grasp(T_world_grasp, planner, gripper):
    """
    Perform a pick and place procedure for the object. One strategy (which we have
    provided some starter code for) is to
    1. Move the gripper from its starting pose to some distance behind the object
    2. Move the gripper to the grasping pose
    3. Close the gripper
    4. Move up
    5. Place the object somewhere on the table
    6. Open the gripper. 

    As long as your procedure ends up picking up and placing the object somewhere
    else on the table, we consider this a success!

    HINT: We don't require anything fancy for path planning, so using the MoveIt
    API should suffice. Take a look at path_planner.py. The `plan_to_pose` and
    `execute_plan` functions should be useful. If you would like to be fancy,
    you can also explore the `compute_cartesian_path` functionality described in
    http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
    
    Parameters
    ----------
    T_world_grasp : 4x4 :obj:`numpy.ndarray`
        pose of gripper relative to world frame when grasping object
    """
    def close_gripper():
        """closes the gripper"""
        gripper.close(block=True)
        rospy.sleep(1.0)

    def open_gripper():
        """opens the gripper"""
        gripper.open(block=True)
        rospy.sleep(1.0)

    inp = raw_input('Press <Enter> to move, or \'exit\' to exit')
    if inp == "exit":
        return

    raise NotImplementedError
    # Go behind object
    pose = Pose()
    pose.position.x = ...
    pose.position.y = ...
    pose.position.z = ...
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion_from_matrix(...)
    plan = planner.plan_to_pose(pose)
    planner.execute_plan(plan)
    
    # Then swoop in
    planner.execute_plan(plan)

    # Bring the object up
    planner.execute_plan(plan)

    # And over
    planner.execute_plan(plan)

    # And now place it
    planner.execute_plan(plan)
    open_gripper()

    raise NotImplementedError

def locate_cube(camera_image_topic, camera_info_topic, camera_frame):
    """
    Finds size and pose of cube in field of view.
    We are leaving this very open ended! Feel free to be creative!
    OpenCV will probably be useful. You may want to look for the
    corners of the cube, or its edges. From that, try your best to reconstruct
    the actual size and pose of the cube!

    Parameters
    ----------
    camera_image_topic : string
        ROS topic for camera image
    camera_info_topic : string
        ROS topic for camera info
    camera_frame : string
        Name of TF frame for camera

    Returns
    -------
    :obj:`trimesh.primatives.Box` : trimesh object for reconstructed cube
    """
    bridge = CvBridge()
    image = rospy.wait_for_message(Image, camera_image_topic)
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    info = rospy.wait_for_message(CameraInfo, camera_info_topic)
    T_world_camera = lookup_transform(camera_frame)

    # Do your image processing on cv_image here!

    side_length = ... # length of one side of cube
    pose = ... # 4x4 homogenous transform for center of cube
    return trimesh.primitives.Box((side_length, side_length, side_length), pose)

def parse_args():
    """
    Parses arguments from the user. Read comments for more details.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-obj', type=str, default='pawn', help=
        """Which Object you\'re trying to pick up.  Options: nozzle, pawn, cube.  
        Default: pawn"""
    )
    parser.add_argument('-n_vert', type=int, default=1000, help=
        'How many vertices you want to sample on the object surface.  Default: 1000'
    )
    parser.add_argument('-n_facets', type=int, default=32, help=
        """You will approximate the friction cone as a set of n_facets vectors along 
        the surface.  This way, to check if a vector is within the friction cone, all 
        you have to do is check if that vector can be represented by a POSITIVE 
        linear combination of the n_facets vectors.  Default: 32"""
    )
    parser.add_argument('-n_grasps', type=int, default=500, help=
        'How many grasps you want to sample.  Default: 500')
    parser.add_argument('-n_execute', type=int, default=5, help=
        'How many grasps you want to execute.  Default: 5')
    parser.add_argument('-metric', '-m', type=str, default='compute_force_closure', help=
        """Which grasp metric in grasp_metrics.py to use.  
        Options: compute_force_closure, compute_gravity_resistance, compute_robust_force_closure"""
    )
    parser.add_argument('-arm', '-a', type=str, default='right', help=
        'Options: left, right.  Default: right'
    )
    parser.add_argument('-robot', type=str, default='baxter', help=
        """Which robot you're using.  Options: baxter, sawyer.  
        Default: baxter"""
    )
    parser.add_argument('--sim', action='store_true', help=
        """If you don\'t use this flag, you will only visualize the grasps.  This is 
        so you can run this outside of hte lab"""
    )
    parser.add_argument('--debug', action='store_true', help=
        'Whether or not to use a random seed'
    )
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()

    if args.debug:
        np.random.seed(0)

    if not args.sim:
        # Init rospy node (so we can use ROS commands)
        rospy.init_node('dummy_tf_node')

    if args.obj != 'cube':
        # Mesh loading and pre-processing
        mesh = trimesh.load_mesh("objects/{}.obj".format(args.obj))
        # Transform object mesh to world frame
        T_world_obj = lookup_transform(args.obj) 
        mesh.apply_transform(T_world_obj)
        mesh.fix_normals()
    else:
        camera_frame = ''
        if robot == 'baxter':
            camera_topic = '/cameras/left_hand_camera/camera_info'
            camera_info = '/cameras/left_hand_camera/camera_info'
            camera_frame = '/left_hand_camera'
        elif robot == 'sawyer':
            camera_topic = '/usb_cam/image_raw'
            camera_info = '/usb_cam/camera_info'
            camera_frame = '/usb_cam'
        else:
            print("Unknown robot type!")
            rospy.shutdown()
        mesh = locate_cube(camera_topic, camera_info, camera_frame)

    # This policy takes a mesh and returns the best actions to execute on the robot
    grasping_policy = GraspingPolicy(
        args.n_vert, 
        args.n_grasps, 
        args.n_execute, 
        args.n_facets, 
        args.metric
    )

    # Each grasp is represented by T_grasp_world, a RigidTransform defining the 
    # position of the end effector
    grasp_vertices_total, grasp_poses = grasping_policy.top_n_actions(mesh, args.obj)

    if not args.sim:
        # Execute each grasp on the baxter / sawyer
        if args.robot == "baxter":
            gripper = baxter_gripper.Gripper(args.arm)
            planner = PathPlanner('{}_arm'.format(args.arm))
        elif args.robot == "sawyer":
            gripper = sawyer_gripper.Gripper("right")
            planner = PathPlanner('{}_arm'.format("right"))
        else:
            print("Unknown robot type!")
            rospy.shutdown()

    for grasp_vertices, grasp_pose in zip(grasp_vertices_total, grasp_poses):
        grasping_policy.visualize_grasp(mesh, grasp_vertices, grasp_pose)
        if not args.sim:
            repeat = True
            while repeat:
                execute_grasp(grasp_pose, planner, gripper)
                repeat = raw_input("repeat? [y|n] ") == 'y'
