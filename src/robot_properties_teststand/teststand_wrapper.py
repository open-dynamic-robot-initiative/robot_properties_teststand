import numpy as np
import time
import os
import pybullet
from bullet_utils.wrapper import PinBulletWrapper
from robot_properties_teststand.config import TeststandConfig
from robot_properties_teststand.utils import find_paths

dt = 1e-3

class TeststandRobot(PinBulletWrapper):

    def __init__(self, pos=None, orn=None, fixed_height=None):

        # Load the robot
        if pos is None:
            pos = [0.0, 0, 0.40]
        if orn is None:
            orn = pybullet.getQuaternionFromEuler([0, 0, 0])

        pybullet.setAdditionalSearchPath(TeststandConfig.paths["package"])
        self.urdf_path = TeststandConfig.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            pos, orn,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=False,
        )
        self.pin_robot = TeststandConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = pybullet.getNumJoints(self.robotId)

        for ji in range(num_joints):
            pybullet.changeDynamics(
                self.robotId,
                ji,
                linearDamping=0.04,
                angularDamping=0.04,
                restitution=0.0,
                lateralFriction=0.5,
            )

        self.base_link_name = "base_link"
        controlled_joints = ["joint_z", "HFE", "KFE"]
        self.joint_names = controlled_joints

        # Creates the wrapper by calling the super.__init__.
        super(TeststandRobot, self).__init__(
            self.robotId, self.pin_robot, controlled_joints, ["END"], useFixedBase=True
        )

        self.nb_dof = self.nv

        if fixed_height:
            pybullet.createConstraint(
                self.robotId,
                0,
                -1,
                -1,
                pybullet.JOINT_FIXED,
                [0, 0, 0],
                [0, 0, 0.0],
                [0, 0, fixed_height],
            )

    def forward_robot(self, q=None, dq=None):
        if not q:
            q, dq = self.get_state()
        elif not dq:
            raise ValueError("Need to provide q and dq or non of them.")

        self.pin_robot.forwardKinematics(q, dq)
        self.pin_robot.computeJointJacobians(q)
        self.pin_robot.framesForwardKinematics(q)
        self.pin_robot.centroidalMomentum(q, dq)

