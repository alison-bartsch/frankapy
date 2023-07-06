from frankapy import FrankaArm, FrankaConstants
import time
import numpy as np

if __name__ == '__main__':
    fa = FrankaArm()

    # start = True
    # if start:
    #     fa.reset_joints()

    #     pose = fa.get_pose()
    #     print(pose)

    #     pose.translation = np.array([0.6, 0, 0.1])
    #     fa.goto_pose(pose)
    #     time.sleep(5)
    print('begin grasps')

    n_grasps = 20
    for i in range(n_grasps):
        fa.goto_gripper(0, grasp=True, force = 10)
        time.sleep(5)
        fa.open_gripper()
        time.sleep(5)
        print('finished grasp', i+1, "out of", n_grasps)