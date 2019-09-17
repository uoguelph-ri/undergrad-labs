import socket
from time import sleep
from colorama import Fore, Style # Adds colour to console output
from robot_loc_world import RobotLocWorld
from robot_loc_joint import RobotLocJoint

SERVER_IP = "192.168.1.1"
ROBOT_IP = "192.168.1.100"
TCP_PORT = 64555
ROBOT_MOTION_DELAY = 0.1

class FanucPCDKClient:
    """Establishes connection between client, server, and robot while including various methods for robot"""
    def __init__(self, print_server_output = False):
        self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buffer_size = 1024
        self.debug_output = print_server_output

    def set_print_debug_output(self, debug_output):
        self.debug_output = debug_output

    # Connects to the PCDK Server at ip:port and instructs the server to connect to the robot at robot_ip
    def connect(self, server_ip = SERVER_IP, port_number = TCP_PORT, robot_ip = ROBOT_IP):
        # Connect to TCP IP address of server and TCP Port
        self.mySocket.connect((server_ip, port_number))

        # Instruct Fanuc server to connect to robot via robot's IP address
        send_message = "connect, " + robot_ip + ",  \r\n"
        response = self.send(send_message)

        print("Connecting to robot at {}".format(server_ip))

        if self.debug_output == True: 
            print("Response: " + response)
        
        if response[0] == "1":
            print("Robot Status: " + Fore.GREEN + "Connected\n" + Style.RESET_ALL)
        else:
            print("Robot Status: " + Fore.RED + "Connection Failed\n" + Style.RESET_ALL)
        
        
    def send(self, data):
        if "readcurposition" not in data[:-1] and self.debug_output == True:
            print("Sending " + data[:-1])

        # Encode data string into stream of bytes and send to server
        self.mySocket.send(data.encode())

        # Return response from server
        return self.mySocket.recv(self.buffer_size).decode()

    def close_connection(self):
        print("Terminating connection to Fanuc PCDK server.")
        self.mySocket.close()

    # Requests the server to run a teach pendant program
    def run_program(self, program_name):
        send_message = "runprogram, " + program_name + " \r\n"
        response = self.send(send_message)

        if self.debug_output == True: print("run_program returnVal: " + response)

    # Requests the server to reset all the robot alarms/errors
    def reset_alarms(self):
        send_message = "resetalarms, \r\n"
        response = self.send(send_message)
        if self.debug_output == True: print("reset_alarms returnVal: " + response)

    # Requests the server to move the robot to world position specified by loc
    def move_world(self, loc):
        send_message = "move, {}, {}, {}, {}, {}, {}, {}, {}, {}, \r\n".format(loc.x, loc.y, loc.z, loc.w, loc.p, loc.r, loc.f, loc.u, loc.t)
        response = self.send(send_message)
        if self.debug_output == True: print("move_world returnVal: " + response)

    # Requests the server to move the robot to joint position specified by loc
    def move_joint(self, loc):
        send_message = "moveJoint, {}, {}, {}, {}, {}, {}, \r\n".format(loc.j1, loc.j2, loc.j3, loc.j4, loc.j5, loc.j6)
        response = self.send(send_message)
        if self.debug_output == True: print("move_joint returnVal: " + response)

    # Request the server to move the robot via linear motion to world position specified by loc
    def move_straight(self, loc):
        send_message = "moveStraight, {}, {}, {}, {}, {}, {}, {}, {}, {}, \r\n".format(loc.x, loc.y, loc.z, loc.w, loc.p, loc.r, loc.f, loc.u, loc.t)
        response = self.send(send_message)
        if self.debug_output == True: print("move_straight returnVal: " + response)

    # Request the value of a specific robot register
    def read_register(self, regNum):
        send_message = "readregister, {}, \r\n".format(regNum)
        response = self.send(send_message)
        if self.debug_output == True: print("read_register returnVal: " + response)

    # Write a value to a specific robot register
    def write_register(self, regNum, value):
        send_message = "writeregister, {}, {}, \r\n".format(regNum, value)
        response = self.send(send_message)
        if self.debug_output == True: print("write_register returnVal: " + response)

    # Request the robot's current position in a specific format (0 for XYZWPR, 1 for Joint)
    def get_position(self, position_format):
        if position_format == "world": position_format = 0
        elif position_format == "joint": position_format = 1
        
        send_message = "readcurposition, {}, \r\n".format(position_format)
        response = self.send(send_message)

        if response != "":
            robot_location = self.parse_loc(response, position_format)
            return robot_location
        else:
            print("get_position received invalid response from PCDK server.")
            return -1

    # Returns the current speed of the robot
    def get_speed(self):
        send_message = "getspeed, \r\n"
        response = self.send(send_message)
        if self.debug_output == True: print("get_speed returnVal: " + response)

        if response != "":
            data_list = response.split(', ')  # Parse response from Fanuc PCDK server into list of strings

            # Verify good data and return speed
            if data_list[0] == "1":
                return int(data_list[1])
        else:
            print("get_speed received invalid response from PCDK server.")
            return -1

    # Sets the speed of the robot
    def set_speed(self, speed):
        send_message = "setspeed, {}, \r\n".format(speed)
        response = self.send(send_message)
        if self.debug_output == True: print("set_speed returnVal: " + response)

    # Unactuates the selected gripper
    def open_gripper(self):
        self.set_gripper(0) # Release object
        
    # Actuates the selected gripper
    def close_gripper(self):
        self.set_gripper(1) # Grasping object

    # Turns robot air on or off
    # gripper_pos: 0 for off, 1 for on
    def set_gripper(self, gripper_pos):
        send_message = "setsuction, {}, \r\n".format(gripper_pos)
        response = self.send(send_message)
        if self.debug_output == True: print("set_suction returnVal: " + response)

    # Converts a string response from PCDK server into RobotLoc coordinates
    # position_format: 0 for world XYZWPR, 1 for joint
    def parse_loc(self, data, position_format):
        data_list = data.split(', ')  # Parse response from Fanuc PCDK server into list of strings

        if data_list[0] == "1":  # Verify good response from server
            if position_format == 0:
                x = float(data_list[1])
                y = float(data_list[2])
                z = float(data_list[3])
                w = float(data_list[4])
                p = float(data_list[5])
                r = float(data_list[6])
                f = data_list[7]
                u = data_list[8]
                t = data_list[9]

                loc_world = RobotLocWorld(x, y, z, w, p, r, f, u, t)
                return loc_world
            elif position_format == 1:
                j1 = float(data_list[1])
                j2 = float(data_list[2])
                j3 = float(data_list[3])
                j4 = float(data_list[4])
                j5 = float(data_list[5])
                j6 = float(data_list[6])


                loc_joint = RobotLocJoint(j1, j2, j3, j4, j5, j6)
                return loc_joint
            else:
                print("Error: incorrect position_format selected. Enter 0 for world XYZWPR, 1 for joint.")
        else:
            print("Error: bad location data returned from PCDK server.")

    """
    Compares your the current robot position with target destination.

    Inputs:
    - cur_pos: current robot location
    - target_pos: where you are sending the robot
    - position_format: "world" or "joint"
    """
    def compare_locations(self, cur_pos, target_pos, position_format, distTolerance = 5, angleTolerance = 2):
        if position_format == "world":
            x = abs(cur_pos.x - target_pos.x)
            y = abs(cur_pos.y - target_pos.y)
            z = abs(cur_pos.z - target_pos.z)
            w = abs(abs(cur_pos.w) - abs(target_pos.w))
            p = abs(cur_pos.p - target_pos.p)
            r = abs(cur_pos.r - target_pos.r)

            if(x > distTolerance or y > distTolerance or z > distTolerance or 
            w > angleTolerance or p > angleTolerance or r > angleTolerance):
                return False
            else:
                return True
        elif position_format == "joint":
            j1 = abs(cur_pos.j1 - target_pos.j1)
            j2 = abs(cur_pos.j2 - target_pos.j2)
            j3 = abs(cur_pos.j3 - target_pos.j3)
            j4 = abs(cur_pos.j4 - target_pos.j4)
            j5 = abs(cur_pos.j5 - target_pos.j5)
            j6 = abs(cur_pos.j6 - target_pos.j6)
            
            if(j1 > angleTolerance or j2 > angleTolerance or 
            j3 > angleTolerance or j4 > angleTolerance or 
            j5 > angleTolerance or j6 > angleTolerance):
                return False
            else:
                return True
        else:
            print("Error, compare_locations() accepts a position_format of 'world'/'joint' only.")

    '''
    Moves the robot using Joint, World, or Linear movements

    Input:
        - loc: location to move to in world or joint coordinate space
        - move_linear: flag for setting whether to move via a linear motion (requires world coord)
        - speed: optional param for adjusting speed only for this move
    '''
    def move(self, loc, delay = ROBOT_MOTION_DELAY, move_linear=False, speed=None):
        # Adjust speed if not None
        if speed:
            prev_speed = self.get_speed()
            self.set_speed(speed)

        if isinstance(loc, RobotLocWorld):
            # Perform world move
            cur_pos = self.get_position("world")
            if self.compare_locations(cur_pos, loc, "world") == False:
                if move_linear:
                    self.move_straight(loc)
                else:
                    self.move_world(loc)

                # Stall program until robot reaches location
                while self.compare_locations(cur_pos, loc, "world") == False:
                    cur_pos = self.get_position("world")
                    sleep(delay)
            else:
                print("> Already at location; skipping move")
        elif isinstance(loc, RobotLocJoint):
            # Perform joint move
            cur_pos = self.get_position("joint")
            if self.compare_locations(cur_pos, loc, "joint") == False:
                self.move_joint(loc)

                # Stall program until robot reaches location
                while self.compare_locations(cur_pos, loc, "joint") == False:
                    cur_pos = self.get_position("joint")
                    sleep(delay)
            else:
                print("> Already at location; skipping move")
        else:
            print(Fore.RED + "[Error] Invalid location or robot is not connected, skipping move_robot() method call." + Style.RESET_ALL)

        # Reset speed back to what it was before
        if speed:
            self.set_speed(prev_speed)
