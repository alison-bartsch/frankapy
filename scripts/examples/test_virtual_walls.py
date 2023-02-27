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

if __name__ == "__main__":
  """
  This function is used to test when the virtual walls are triggered.
  """
    fa = FrankaArm()
    print("Initial: ", fa.get_pose())
    # fa.reset_pose()
    fa.reset_joints()

    y = np.linspace(0,0.79,10)
    pose = fa.get_pose()

    start_pose = pose
    print('start pose: ', start_pose)

    for i in y:
    	start_pose.translation = [0.4+i, 0, 0.35]
    	print('\nedited start pose: ', start_pose)
    	fa.goto_pose(start_pose)

    print('finished')