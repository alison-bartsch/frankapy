from autolab_core import RigidTransform
from frankapy import FrankaArm
from frankapy.vision_utils import *
import time


if __name__ == "__main__":
    
    fa = FrankaArm()
    fa.reset_joints()

    # fa.open_gripper()
    # time.sleep(5)
    fa.close_gripper()
