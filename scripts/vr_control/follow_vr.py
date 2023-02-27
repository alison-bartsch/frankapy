import pickle as pkl
import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

import rospy
import time

# from perception_utils.realsense import get_first_realsense_sensor
from franka_gripper.msg import *
from actionlib import SimpleActionClient


if __name__ == "__main__":
    """
    This file takes an end-effector trajectory collected in VR and plays through
    it smoothly.
    """

    fa = FrankaArm()
    fa.reset_joints()

    # load in the trajectory - should be replaced for final version
    pose_traj = pkl.load(open('franka_full_oculus_data_test.pkl','rb'))
    print("Pose Traj. Size: ", len(pose_traj))

    offset = np.array([0, 0, 0.025])
    block_width = 0.03
    T = 10
    dt = T/np.shape(pose_traj)[0]
    ts = np.arange(0, T, dt)

    pose = fa.get_pose()
    print("Robot Resting Pose: ", pose)

    # initialize the publisher
    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    rate = rospy.Rate(1 / dt)

    start_pose = pose 
    start_pose.translation = pose_traj[1][0:3] + offset

    # move the robot to the initial location of the trajectory
    rospy.loginfo('Publishing pose trajectory...')

    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(start_pose)
    fa.goto_pose(start_pose, duration=(T), dynamic=True, buffer_time=10,
        cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
    )
    fa.goto_gripper(pose_traj[1][3])
    print("\nWent to initial pose!")

    init_time = rospy.Time.now().to_time()
    for i in range(2, len(ts)):
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, timestamp=timestamp,
            position=pose_traj[i][0:3]*np.array([1, 1, 1]) + offset, quaternion=pose.quaternion
		)
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
            )

        min_gripper_delay = 0.2
        gripper_update_rate = 1 + int(min_gripper_delay/dt)
        if i % gripper_update_rate == 0:
            difference = pose_traj[i][3] - fa.get_gripper_width()
            speed = abs(difference/(dt*gripper_update_rate ))
            print("\nSpeed: ", speed)
            if speed > 0.15:
                speed = 0.15
            if abs(difference) > 0.001:
                if difference >= 0:
                    print("opening")
                    fa.goto_gripper(pose_traj[i][3], block=False, grasp=False, speed=speed)
                else:
                    print("closing")
                    fa.goto_gripper(pose_traj[i][3], block=False, grasp=True, speed=speed)
                print("Desired Gripper Width: ", pose_traj[i][3], "Current Gripper State: ", fa.get_gripper_width())
            else:
                print("do nothing :)")

        rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
        pub.publish(ros_msg)
        rate.sleep()

    # fa.goto_gripper(0.05, block=True, grasp=False, speed=0.04)

    fa.stop_gripper()
    print('stop gripper')
    fa.open_gripper()
    print('start sleep')
    time.sleep(5)

    # Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)

    print("Gripper State: ", fa.get_gripper_width())

    rospy.loginfo('Done')

    print("Gripper State: ", fa.get_gripper_width())
