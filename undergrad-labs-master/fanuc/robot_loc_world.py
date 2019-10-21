class RobotLocWorld:
    """Defines world coordinates of Fanuc robot"""
    def __init__(self, x, y, z, w, p, r, f, u, t):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.p = p
        self.r = r
        self.f = f
        self.u = u
        self.t = t

    # Returns xyz positions of a defined world location
    def get_world_xyz(self):
        return self.x, self.y, self.z

    # Returns xyzwpr position of a defined world location as an array
    def get_loc_as_array(self):
        # format to 3 decimal places and return as array
        x = "{:.3f}".format(self.x)
        y = "{:.3f}".format(self.y)
        z = "{:.3f}".format(self.z)
        w = "{:.3f}".format(self.w)
        p = "{:.3f}".format(self.p)
        r = "{:.3f}".format(self.r)

        return [x, y, z, w, p, r, self.f, self.u, self.t]

    # Overwrites str(obj): when calling str(RobotLocWorld), will automatically return string in format given below
    def __str__(self):
        return "x={:.3f}; y={:.3f}; z={:.3f}; w={:.3f}; p={:.3f}; r={:.3f}; f={}; u={}; t={}".format(self.x, self.y, self.z, self.w, self.p, self.r, self.f, self.u, self.t)

