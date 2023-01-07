#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("base_link",
               "gripper_tip_link")

seed_state = [0.0] * ik_solver.number_of_joints

result = ik_solver.get_ik(seed_state,
                0.50, -0.1, 0.8,  # X, Y, Z
                0.0, 0.0, 0.0, 1.0)  # QX, QY, QZ, QW

print(result)
