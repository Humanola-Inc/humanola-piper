import math
import pathlib
from dataclasses import dataclass
from typing import Tuple

import casadi
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin

URDF_PATH = pathlib.Path("/app/URDF/piper.urdf")
MESH_PATH = pathlib.Path("/app/URDF")


def matrix_to_xyzrpy(matrix: np.ndarray) -> list[float]:
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def create_transformation_matrix(
    x: float, y: float, z: float, roll: float, pitch: float, yaw: float
) -> np.ndarray:
    transformation_matrix = np.eye(4)
    a = np.cos(yaw)
    b = np.sin(yaw)
    c = np.cos(pitch)
    d = np.sin(pitch)
    e = np.cos(roll)
    f = np.sin(roll)
    de = d * e
    df = d * f
    transformation_matrix[0, 0] = a * c
    transformation_matrix[0, 1] = a * df - b * e
    transformation_matrix[0, 2] = b * f + a * de
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = b * c
    transformation_matrix[1, 1] = a * e + b * df
    transformation_matrix[1, 2] = b * de - a * f
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -d
    transformation_matrix[2, 1] = c * f
    transformation_matrix[2, 2] = c * e
    transformation_matrix[2, 3] = z
    return transformation_matrix


def calc_pose_incre(base_pose: list[float], pose_data: list[float]) -> list[float]:
    begin_matrix = create_transformation_matrix(
        base_pose[0],
        base_pose[1],
        base_pose[2],
        base_pose[3],
        base_pose[4],
        base_pose[5],
    )
    zero_matrix = create_transformation_matrix(0.19, 0.0, 0.2, 0, 0, 0)
    end_matrix = create_transformation_matrix(
        pose_data[0],
        pose_data[1],
        pose_data[2],
        pose_data[3],
        pose_data[4],
        pose_data[5],
    )
    result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(begin_matrix), end_matrix))
    return matrix_to_xyzrpy(result_matrix)


def quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    qw = math.sqrt(1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2]) / 2
    qx = (matrix[2, 1] - matrix[1, 2]) / (4 * qw)
    qy = (matrix[0, 2] - matrix[2, 0]) / (4 * qw)
    qz = (matrix[1, 0] - matrix[0, 1]) / (4 * qw)
    return np.array([qx, qy, qz, qw])


@dataclass
class PiperSolverConfig:
    ground_height: float = 0.0


class PiperSolver:
    def __init__(self, config: PiperSolverConfig):
        self.config = config
        self.robot = pin.RobotWrapper.BuildFromURDF(
            str(URDF_PATH), package_dirs=str(MESH_PATH)
        )
        self.mixed_joints_to_lock_ids = ["joint7", "joint8"]
        self.robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_joints_to_lock_ids,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        self.ground_height = self.config.ground_height
        self.exclude_from_ground_collision = ["base_link_0"]
        self.gripper_id = self.robot.model.addFrame(
            pin.Frame(
                "ee",
                self.robot.model.getJointId("joint6"),
                pin.SE3(
                    np.eye(3),
                    np.array([0, 0, 0.13]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )

        # self.geometry_data = pin.GeometryData(self.geom_model)
        # self.history_data = np.zeros(self.robot.model.nq)

        self.cmodel = cpin.Model(self.robot.model)
        self.cdata = self.cmodel.createData()
        self.cq = casadi.SX.sym("q", 6, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(
                        self.cdata.oMf[self.gripper_id].inverse() * cpin.SE3(self.cTf)
                    ).vector,
                )
            ],
        )

        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(6)
        self.prev_var_q = self.opti.parameter(6)
        self.param_tf = self.opti.parameter(4, 4)

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3]
        ori_error = error_vec[3:]
        weight_position = 1.0
        weight_orientation = 0.1
        self.totalcost = casadi.sumsqr(weight_position * pos_error) + casadi.sumsqr(
            weight_orientation * ori_error
        )
        self.regularization = casadi.sumsqr(self.var_q)
        self.opti.subject_to(
            self.opti.bounded(
                self.robot.model.lowerPositionLimit,
                self.var_q,
                self.robot.model.upperPositionLimit,
            )
        )
        self.opti.minimize(
            20 * self.totalcost
            + 0.01 * self.regularization
            + 0.2 * casadi.sumsqr(self.var_q - self.prev_var_q)
        )

        opts = {
            "ipopt": {"print_level": 0, "max_iter": 20, "tol": 1e-2},
            "print_time": False,
            "jit": True,
            "compiler": "shell",
            "jit_options": {
                "flags": ["-Ofast", "-march=native"],
                "compiler": "gcc",
            },
            "jit_cleanup": True,
            "expand": True,
        }
        self.opti.solver("ipopt", opts)

        # Bake the whole solve into one reusable, JIT-compiled function:
        #   (q_init, tf, prev_q) -> q_sol   (var_q doubles as the initial guess)
        self.solve_fn = self.opti.to_function(
            "ik",
            [self.var_q, self.param_tf, self.prev_var_q],
            [self.var_q],
            ["q_init", "tf", "prev_q"],
            ["q_sol"],
        )

        # Warm-up: triggers JIT compilation now, so the first real call isn't stalled.
        target = self.forward(np.zeros(6))
        self.inverse(target, np.zeros(6))

    def inverse(
        self, v: np.ndarray, prev_joints: np.ndarray
    ) -> Tuple[float, float, float, float, float, float]:
        sol_q = self.solve_fn(np.zeros(6), v, prev_joints)
        return tuple(float(q) for q in np.array(sol_q).flatten())  # type: ignore[return-value]

    def forward(self, joints: np.ndarray) -> np.ndarray:
        data = self.robot.model.createData()
        pin.forwardKinematics(self.robot.model, data, joints)
        pin.updateFramePlacements(self.robot.model, data)
        return data.oMf[self.gripper_id].homogeneous
