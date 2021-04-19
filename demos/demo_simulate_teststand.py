#!/usr/bin/env python

"""demo_simulate_teststand

Simple demo showing how the simulation setup works.

License: BSD 3-Clause License
Copyright (C) 2018-2019, New York University , Max Planck Gesellschaft
Copyright note valid unless otherwise stated in individual files.
All rights reserved.
"""

import time
import numpy as np
import pybullet as p
from bullet_utils.env import BulletEnvWithGround
from robot_properties_teststand.teststand_wrapper import TeststandRobot, TeststandConfig

if __name__ == "__main__":

    # Create a robot instance. This initializes the simulator as well.
    env = BulletEnvWithGround(p.GUI)
    robot = TeststandRobot(fixed_height=True)
    env.add_robot(robot)
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = np.matrix(TeststandConfig.initial_configuration).T
    dq0 = np.matrix(TeststandConfig.initial_velocity).T
    robot.reset_state(q0, dq0)

    # Run the simulator for 100 steps
    for i in range(500000):
        # TODO: Implement a controller here.
        robot.send_joint_command(tau)

        # Step the simulator.
        robot.step_simulation()
        time.sleep(0.001)  # You can sleep here if you want to slow down the replay

    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    active_eff, forces = robot.get_force()
    print("q", q)
    print("dq", dq)
    print("active eff", active_eff)
    print("forces", forces)
