# Import mavutil
from pymavlink import mavutil
from math import isclose

class HaloAUV:
    def __init__(self):
        # ===== Set up Mavlink Comms ===== #
        # Create the connection to the top-side computer as companion computer/autopilot
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()

        # Initialize mode 
        self.set_mode()
        
        # Save depth and have a class variable to keep track of depth
        self.current_depth = self.get_depth()
        
        # Set Kp gain
        self.Kp = 10

    def arm(self):
        """Arm the robot.
        """
        # Send message to arm vehicle
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

        # Confirm that vehicle is armed
        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        print('Armed!')

    def set_mode(self, mode = "MANUAL"):
        """Set the mode of the robot (i.e. Manual, Stabilize, etc). Modes are standard Ardusub modes

        Args:
            mode (str, optional): Mode you want to set the vehicle to. Defaults to "MANUAL".
        """
        print("ArduSub mode = ", mode)
        # Check if mode is available
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)

        # Get mode ID
        mode_id = self.master.mode_mapping()[mode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)

    def get_depth(self):
        """Get depth from barometer.

        Returns:
            float: Robot's current depth
        """
        print("Waiting for depth reading")
        read_flag = True
        while read_flag:
            msg = self.master.recv_match()
            if not msg:
                continue
            if msg.get_type() == 'VFR_HUD':
                print("\n** Recieved depth **")
                print("Depth: %s" % msg.to_dict()["alt"])
                read_flag = False

        # Update current position everytime you read depth
        self.current_depth = msg.to_dict()["alt"]
        return self.current_depth

    def set_relative_depth(self, diff):
        """Move robot up or down based on input relative depth

        Args:
            diff (float): Difference in depth to move the robot up or down in centimeters
        """

        # Set target depth
        target_depth = self.get_depth() + diff
        print("Target depth: ", target_depth)

        # Keep moving robot until is within 1 cm of the target depth
        while(not isclose(target_depth, self.get_depth(), rel_tol = 0.1)):
            print("Moving to desired depth")

            error = target_depth - self.get_depth()
            throttle = 500 + self.Kp*(error)
            print("Throttle  ", throttle)

            # Move robot
            self.ascend(throttle)

        print("Reached desired depth!")

    def restart_pixhawk(self):
        self.master.reboot_autopilot()

    def ascend(self, z_throttle):
        """Move the robot up or down

        Args:
            z_throttle (double): throttle value to move robot (500 for stop, 1000 for full upward, 0 for full descind)
        """

        # Check if input is valid, set to 0% if not and print error
        if(z_throttle > 1000 or z_throttle < 0):
            z_throttle = 500
            print("ERROR: INVLAID THROTTLE INPUT -> STOPPED ROBOT")

        # Cap min motor movement -> it doe snot move at lower throttle
        if(z_throttle < 580 and z_throttle > 500):
            z_throttle = 580

        if(z_throttle > 400 and z_throttle < 500):
            z_throttle = 400


        self.master.mav.manual_control_send(
            self.master.target_system,
            0,
            0,
            int(z_throttle), # Move robot in z axis
            0,
            0,
            0)

    # def tilt_camera(self, angle):


