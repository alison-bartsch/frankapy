#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32MultiArray

import pickle as pkl
import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from frankapy.utils import min_jerk, min_jerk_weight

def callback(data):
	"""
	INSERT DESCRIPTION
	"""
	# rospy.loginfo(rospy.get_caller_id() + "Data: %s", data.data)

	pose_traj = data.data
	timestamp = rospy.Time.now().to_time()

	# timestamp = rospy.Time.now().to_time() - init_time
    traj_gen_proto_msg = PosePositionSensorMessage(
        id=i, timestamp=timestamp,
        position=pose_traj[0:3]*np.array([1, 1, 1]) + np.array([0, 0, 0.05]), quaternion=pose.quaternion
	)
    ros_msg = make_sensor_group_msg(
        trajectory_generator_sensor_msg=sensor_proto2ros_msg(
            traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
        )

    # NOTE: there is some delay due to the speed of moving the gripper, by increasing the speed from standard, we are able to follow trajectory more closely
    fa.goto_gripper(pose_traj[3], block=False, speed=0.1)
    print("\nDesired Gripper Width: ", pose_traj[3], "Current Gripper State: ", fa.get_gripper_width())

    rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
    pub.publish(ros_msg)
    rate.sleep()

def listener():
	"""
	INSERT DESCRIPTION
	"""
	rospy.init_node('vr_subscriber', anonymous=True)
	rospy.Subscriber('pose_and_gripper', Float32MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':

	fa = FrankaArm()
    fa.reset_joints()

    # # load in the trajectory - should be replaced for final version
    # pose_traj = pkl.load(open('demo_scripts/Pick_and_Place_Motion.p','rb'))
    # print("Pose Traj. Size: ", len(pose_traj))

    T = 15
    dt = 0.2
    ts = np.arange(0, T, dt)

    pose = fa.get_pose()
    print("Robot Resting Pose: ", pose)

    # initialize the publisher
    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    rate = rospy.Rate(1 / dt)

    # initialize the subscriber
    # should subscribe to the VR topic & have a simpler custom message, similar queue size

    start_pose = pose
    start_pose.translation = pose_traj[1][0:3]

    # move the robot to the initial location of the trajectory
    rospy.loginfo('Publishing pose trajectory...')
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(start_pose)
    fa.goto_pose(start_pose, duration=T, dynamic=True, buffer_time=10,
        cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
    )
    print("\nWent to initial pose!")

    # enter the loop of subscribing to the vr commands
	listener()

	# Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)

    rospy.loginfo('Done')