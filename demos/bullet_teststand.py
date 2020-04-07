"""
@file bullet_teststand.yaml
@author Maximilien Naveau (maximilien.naveau@gmail.com)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-05-22
@brief Define the interface between the control and the hardware
"""

import os
import time
import rospkg

import numpy as np

import pybullet as p
import pinocchio as se3

from py_robot_properties_teststand.config import TeststandConfig


physicsClient = p.connect(p.GUI)

# Load the plain.
plain_urdf = rospkg.RosPack().get_path("robot_properties_teststand") + \
    "/urdf/plane_with_restitution.urdf"
planeId = p.loadURDF(plain_urdf)

print("Loaded plain.")

# Load the robot
# robotStartPos = [0.,0,0.40]
# robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

urdf_path = TeststandConfig.urdf_path
robotId = p.loadURDF(urdf_path, useFixedBase=True,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)
p.getBasePositionAndOrientation(robotId)

# Create the robot wrapper in pinocchio.
pin_robot = TeststandConfig.buildRobotWrapper()

# Query all the joints.
num_joints = p.getNumJoints(robotId)

for ji in range(num_joints):
    p.changeDynamics(robotId, ji, linearDamping=.04,
                     angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

p.setGravity(0, 0, -9.81)
p.setPhysicsEngineParameter(1e-3, numSubSteps=1)

# Need to turn-off the default velocity controller.
p.setJointMotorControlArray(
    robotId, [0, 1, 2], p.VELOCITY_CONTROL, forces=np.zeros(3))

# Initial configuration for the robot.
q = np.array([0.3, 0.8, -1.6])
dq = np.zeros(3)


def setState(q, dq):
    for i in range(3):
        p.resetJointState(robotId, i, q[i], dq[i])


setState(q, dq)

raw_input("Press enter to start simulation")

# Run the simulation for 5000 steps = 5 seconds.
for i in range(5000):
    p.stepSimulation()
    time.sleep(5e-4)

raw_input("Press enter to quit")
