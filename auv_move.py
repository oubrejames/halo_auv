# Import mavutil
from pymavlink import mavutil
from math import isclose

class HaloAUV:
    """_summary_
    """
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
        
        # Set PI gains for depth
        self.Kp_depth = 2.0
        self.Ki_depth = 0.01
        
        # Set PI gains for x
        self.Kp_x = 2.0
        self.Ki_x = 0.01
        
        # Set distance you want robot to go in front april tag
        self.dist_to_tag = 150 # About 6 inches

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
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                print("\n** Recieved depth **")
                read_flag = False

        # Update current position everytime you read depth
        self.current_depth = msg.to_dict()["relative_alt"]

        return self.current_depth

    def set_relative_depth(self, diff):
        """Move robot up or down based on input relative depth

        Args:
            diff (float): Difference in depth to move the robot in millimeters
        """

        # Set target depth
        self.get_depth()

        target_depth = self.current_depth + diff

        # Keep moving robot until is within 1 cm of the target depth
        sum_error = 0
        while(not isclose(target_depth, self.get_depth(), rel_tol = 0.01)):
            print("Moving to desired depth")

            error = target_depth - self.current_depth
            sum_error += error
            if(sum_error > 100):
                sum_error = 100
            if(sum_error < -100):
                sum_error = -100

            throttle = 500 + self.Kp_depth*(error) + self.Ki_depth*(sum_error)
            print("Throttle  ", throttle)

            # Move robot
            self.ascend(throttle)

        print("Reached desired depth!")

    def restart_pixhawk(self):
        self.master.reboot_autopilot()

    def hold_depth(self):
        """TEMP FOR TESTING MUST UPDATE
        """
        self.set_relative_depth(0.0)

    def ascend(self, z_throttle):
        """Move the robot up or down

        Args:
            z_throttle (double): throttle value to move robot up or down (500 for stop, 1000 for full upward, 0 for full descind)
        """
        # Check if input is valid, set to 0% if not and print error
        if(z_throttle > 1000):
            z_throttle = 1000
        
        if(z_throttle < 0):
            z_throttle = 0

        self.master.mav.manual_control_send(
            self.master.target_system,
            0,
            0,
            int(z_throttle), # Move robot in z axis
            0,
            0,
            0)

    def move_x(self, x_throttle):
        """Move forward/backward in the x direction

        Args:
            x_throttle (double): throttle value to move robot in x direction (-1000 for full reverse, 1000 for full forward)
        """
        # Check if input is valid, set to 0% if not and print error
        if(x_throttle > 1000):
            x_throttle = 1000
        
        if(x_throttle < -1000):
            x_throttle = -1000

        self.master.mav.manual_control_send(
            self.master.target_system,
            int(x_throttle), # Move robot in x axis
            0,
            0,
            0,
            0,
            0)

    def set_relative_x(self, target_x):

        error = target_x - self.dist_to_tag
        sum_error += error
        if(sum_error > 100):
            sum_error = 100
        if(sum_error < -100):
            sum_error = -100
            
        # Keep moving robot until is within 1 cm of the target depth
        sum_error = 0
        while(not isclose(target_x, self.dist_to_tag, rel_tol = 0.01)):
            print("Moving to desired x position")

            throttle = self.Kp_x*(error) + self.Ki_x*(sum_error)
            print("Throttle  ", throttle)

            # Move robot
            self.move_x(throttle)

        print("Reached desired x position!")

    # def tilt_camera(self, angle):


