#!/usr/bin/env python

from fanuc_pcdk_client import FanucPCDKClient
from robot_loc_world import RobotLocWorld
from robot_loc_joint import RobotLocJoint
from time import sleep

ROBOT_SPEED = 30
ROBOT_SPEED_LINEAR = 45

# Robot init
robot = FanucPCDKClient()
robot.connect()

# Run teach pendant program.
robot.run_program('PCDK')
robot.reset_alarms()

# Adjust the speed of the robot
robot.set_speed(ROBOT_SPEED)


# Move robot using joint coordinates
joint_loc = RobotLocJoint(j1=0, j2=0, j3=0, j4=0, j5=0, j6=0)
robot.move_joint(joint_loc)

# Move robot using world coordinates
world_loc = RobotLocWorld(x=100, y=100, z=100,
                          w=178.437, p=-0.195, r=0.001, 
                          f=False, u=True, t=True)
robot.move(world_loc)

# Move robot using linear motion (requires world coordinates)
robot.move(world_loc, move_linear=True, speed=ROBOT_SPEED_LINEAR)


# Open gripper
robot.open_gripper()

# Close Gripper
robot.close_gripper()


# Retrieve the current world XYZWPR coordinates of the robot
loc_world = robot.get_position("world")
print(f'Current world location: {loc_world}\n')

# Retrieve the current joint coordinates of the robot
loc_joint = robot.get_position("joint")
print(f'Current joint location: {loc_joint}\n')


# Close the connection to the Fanuc PCDK server
robot.close_connection()
