class RobotLocJoint:
    """Defines joint coordinates of Fanuc robot"""
    def __init__(self, j1, j2, j3, j4, j5, j6):
        self.j1 = j1
        self.j2 = j2
        self.j3 = j3
        self.j4 = j4
        self.j5 = j5
        self.j6 = j6
    
    # Returns all joint positions of a defined joint location as an array
    def get_loc_as_array(self, round_values=True, return_floats=False):
        if round_values:
            # format to 3 decimal places and return as array
            j1 = "{:.3f}".format(self.j1)
            j2 = "{:.3f}".format(self.j2)
            j3 = "{:.3f}".format(self.j3)
            j4 = "{:.3f}".format(self.j4)
            j5 = "{:.3f}".format(self.j5)
            j6 = "{:.3f}".format(self.j6)
        else:
            # Return as array with all decimal places
            j1 = "{}".format(self.j1)
            j2 = "{}".format(self.j2)
            j3 = "{}".format(self.j3)
            j4 = "{}".format(self.j4)
            j5 = "{}".format(self.j5)
            j6 = "{}".format(self.j6)

        robot_joint_angles = [j1, j2, j3, j4, j5, j6]

        if return_floats:
            for i in range(0, len(robot_joint_angles)):
                robot_joint_angles[i] = float(robot_joint_angles[i])

        return robot_joint_angles

    # Overwrites str(obj): when calling str(RobotLocJoint), will automatically return string in format given below
    def __str__(self):
        return "j1={:.3f}; j2={:.3f}; j3={:.3f}; j4={:.3f}; j5={:.3f}; j6={:.3f}".format(self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)

