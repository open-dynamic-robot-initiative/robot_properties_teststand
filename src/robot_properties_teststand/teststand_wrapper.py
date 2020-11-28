import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory
import pybullet
from py_pinocchio_bullet.wrapper import PinBulletWrapper
from py_robot_properties_teststand.config import TeststandConfig


dt = 1e-3


class TeststandRobot(PinBulletWrapper):
    @staticmethod
    def initPhysicsClient():
        physicsClient = pybullet.connect(pybullet.GUI)
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=dt, numSubSteps=1)
        return physicsClient

    def __init__(self, physicsClient=None, fixed_height=None):
        if physicsClient is None:
            self.physicsClient = self.initPhysicsClient()

        # Load the plain.
        plain_urdf = os.path.join(
            get_package_share_directory("pinocchio_bullet"), "urdf", "ground.urdf"
        )
        self.planeId = pybullet.loadURDF(plain_urdf)

        # Load the robot
        robotStartPos = [0, 0, 0.0]
        robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])

        self.urdf_path = TeststandConfig.urdf_path
        self.robotId = pybullet.loadURDF(
            self.urdf_path,
            robotStartPos,
            robotStartOrientation,
            flags=pybullet.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=False,
        )
        pybullet.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) + "/urdf"]
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

