import numpy as np
import numpy.typing as npt

from .solver import ArmSolver


class ArmState:
    def __init__(self, joints: npt.NDArray[np.float64], solver: ArmSolver):
        self.joints = joints
        self.solver = solver
        self.pose = solver.forward(self.joints[:6])

    def set_with_ef_delta(self, ef_delta: npt.NDArray[np.float64]):
        self.set_with_pose(self.pose @ ef_delta)

    def set_with_pose(self, pose: npt.NDArray[np.float64]):
        self.joints[:6] = self.solver.backward(pose, self.joints[:6])
        self.pose = pose

    def set_with_joints(self, joints: npt.NDArray[np.float64]):
        pose = self.solver.forward(joints[:6])
        self.pose = pose
        self.joints = joints.copy()

    def open_gripper(self):
        self.joints[6] = 1

    def close_gripper(self):
        self.joints[6] = 0
