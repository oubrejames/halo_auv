import rclpy
from rclpy.node import Node
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_path
from pymavlink import mavutil
from math import isclose
from enum import Enum, auto
from apriltag_msgs.msg import AprilTagDetectionArray

class State(Enum):
    """States to keep track of where the system is."""
    SEARCH_FOR_TAG = auto(),
    INIT = auto(),
    MATH_DEPTH = auto()


class HaloControl(Node):
    """
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('auv_camera')

        # ===== Set up Mavlink Comms ===== #
        # Create the connection to the top-side computer as companion computer/autopilot
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()

        # Initialize mode 
        self.set_mode()
        
        # Save depth and have a class variable to keep track of depth
        self.current_depth = self.get_depth()

        # Save heading and have a class variable to keep track of heading (degrees)
        self.current_heading = self.get_heading()
        
        # Set PI gains for depth
        self.Kp_depth = 2.0
        self.Ki_depth = 0.01
        
        # Set PI gains for x
        self.Kp_x = 2.0
        self.Ki_x = 0.01

        # Set PI gains for angle
        self.Kp_a = 2.0
        self.Ki_a = 0.01

        # Set distance you want robot to go in front april tag
        self.dist_to_tag = 150 # About 6 inches
        
        # Create the timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Make subscriber to detections topic to check if april tag is detected
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, "/detections", self.detection_cb, 10)

        # Flag for if you currently see an apriltag
        self.april_flag = False
        self.state = State.INIT

    def timer_callback(self):
        
        # Get apriltag position
        # TODO Add tf lisenter

        if self.state == State.INIT:
            # Arm AUV
            self.arm()

            # Get initial depth reading
            self.get_depth()
            
            # Update state to look for apriltag
            self.state = State.SEARCH_FOR_TAG

        if self.state == State.SEARCH_FOR_TAG:
            # Perform search algorithm
            self.search_for_tag()
            self.state == State.MATCH_DEPTH

    def detection_cb(self, msg):
        """
        """
        if(msg.detections):
            self.april_flag = True
        else:
            self.april_flag = False

    def search_for_tag(self):
        angle_tracker = 0
        while(not self.april_flag):
            # Turn 10 Deg
            self.set_relative_angle(10)
            angle_tracker += 10
            if(angle_tracker > 345):
                # Reset tracker
                angle_tracker = 0

                # Move up 10 cm
                self.set_relative_depth(100)

    def move_rotate(self, r_throttle):
        """
        """
        if(r_throttle > 1000):
            r_throttle = 1000
        
        if(r_throttle < -1000):
            r_throttle = -1000

        self.master.mav.manual_control_send(
            self.master.target_system,
            0,
            0,
            500,
            int(r_throttle),
            0,
            0)

    def set_relative_angle(self, diff):

        target_angle = self.get_heading() + diff

        # Wrap angle - Mavlink uses angles from 0-359.99
        target_angle = self.wrap_angle(target_angle)

        sum_error = 0
        while(not isclose(target_angle, self.get_heading(), rel_tol = 0.01)):
            error = target_angle - self.current_depth
            sum_error += error

            if(sum_error > 100):
                sum_error = 100
            if(sum_error < -100):
                sum_error = -100

            throttle = self.Kp_a*(error) + self.Ki_a*(sum_error)
            # Move robot
            self.move_rotate(throttle)

    def wrap_angle(self, angle):
        if ( angle > 359.99):
            angle -= 359.99

        if (angle < 0.0):
            angle += 359.99

        return angle

    def get_heading(self):
        read_flag = True
        while read_flag:
            msg = self.master.recv_match()
            if not msg:
                continue
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            print("\n** Recieved Heading **")
            read_flag = False

        # Update current position everytime you read heading
        self.current_heading = msg.to_dict()["hdg"]

        return self.wrap_angle(self.current_heading)

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
        # print("Waiting for depth reading")
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

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    halo_control_node = HaloControl()

    # Spin the node so the callback function is called.
    rclpy.spin(halo_control_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    halo_control_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()