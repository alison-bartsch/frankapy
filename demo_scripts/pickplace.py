import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm

"""
This function is a hard-coded demo of the pick and place task
with a block to familiarize with the code-based control. 
"""

if __name__ == "__main__":
    fa = FrankaArm()
    
    # reset franka to its home joints
    fa.reset_joints()