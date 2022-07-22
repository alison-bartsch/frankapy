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

# NOTE: In order to turn this script into live-immitating the VR controller pose/gripper width, we would need
# a subscriber in here that loads in the trajectory step by step. Instead of iterating through pose_traj in a 
# for loop, would have a while loop for while getting commands from VR ros node (or whatever we call it)

# NOTE: It actually would be helpful to get more steps in the trajectory, as one of the issues is that the 
# gripper closing can lag behind because the step size between trajectory points is quite large (e.g. 0.8 to 
# 0.6 in one step), which since the gripper and pose are decoupled can cause the timings to mis-align

if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()

    rospy.loginfo('Generating Trajectory')

    pose_traj = pkl.load(open('demo_scripts/Pick_and_Place_Motion.p','rb'))

    print("Pose Traj. Size: ", len(pose_traj))

    T = 15
    dt = 0.2
    ts = np.arange(0, T, dt)

    print("ts length: ", len(ts))

    pose = fa.get_pose()
    print("Robot Resting Pose: ", pose)
    # assert False

    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    rate = rospy.Rate(1 / dt)

    start_pose = pose
    print('start pose: ', start_pose)
    start_pose.translation = pose_traj[1][0:3]
    print('edited start pose: ', start_pose)

    # assert False

    rospy.loginfo('Publishing pose trajectory...')
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(start_pose)
    fa.goto_pose(start_pose, duration=T, dynamic=True, buffer_time=10,
        cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
    )

    print("\nWent to initial pose!")
    #time.sleep(5)

    init_time = rospy.Time.now().to_time()
    for i in range(2, len(ts)):
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, timestamp=timestamp,
            position=pose_traj[i][0:3]*np.array([1, 1, 1]) + np.array([0, 0, 0.05]), quaternion=pose.quaternion
		)
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
            )

        # NOTE: there is some delay due to the speed of moving the gripper, by increasing the speed from standard, we are able to follow trajectory more closely
        fa.goto_gripper(pose_traj[i][3], block=False, speed=0.1)
        print("\nDesired Gripper Width: ", pose_traj[i][3], "Current Gripper State: ", fa.get_gripper_width())

        rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
        pub.publish(ros_msg)
        rate.sleep()

        
    # Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)

    rospy.loginfo('Done')
