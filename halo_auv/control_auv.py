import rclpy
from rclpy.node import Node
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_path
from pymavlink import mavutil
from math import isclose, acos
from enum import Enum, auto
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

class State(Enum):
    """States to keep track of where the system is."""
    SEARCH_FOR_TAG = auto(),
    INIT = auto(),
    MATCH_DEPTH = auto(),
    GO_TO_TAG = auto(),
    HOLD_POSE = auto(),
    READ_TAG = auto(),
    NOTHING = auto()




class HaloControl(Node):
    """
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('auv_control')

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
        self.Kp_x = 1.0
        self.Ki_x = 0.01

        # Set PI gains for angle
        self.Kp_a = 25.0
        self.Ki_a = 0.03

        # Set distance you want robot to go in front april tag
        self.dist_to_tag = 150 # About 6 inches
        
        # Create the timer
        self.timer = self.create_timer(0.005, self.timer_callback)

        # Make subscriber to detections topic to check if april tag is detected
        # self.detection_sub = self.create_subscription(
        #     AprilTagDetectionArray, "detections", self.detection_cb, 10)

        # Flag for if you currently see an apriltag
        self.april_flag = False
        self.tmp_flag = False
        self.state = State.INIT

        # Create a listener to recieve the TF's from each tag to the camera
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create global april tag position vairable (in april tag frame)
        self.april_x = 0.0
        self.april_y = 0.0
        self.april_z = 0.0
        
        # Variable to track how much you have turned
        self.angle_tracker = 0
        self.prev_april_count = 0
        self.april_count = 0

    def timer_callback(self):

        if self.state == State.INIT:
            self.get_logger().info(f'State = INIT', once=True)
            # Arm AUV
            self.arm()

            # Get initial depth reading
            self.get_depth()
          
            # Update state to look for apriltag
            self.state = State.READ_TAG

        # Only switch states once you have checked if you are seeing an april tag or not
        if self.state == State.READ_TAG:
            self.get_logger().info(f'State = READ_TAG', once=True)
            if(self.april_count > self.prev_april_count):
                self.state = State.SEARCH_FOR_TAG
            self.prev_april_count = self.april_count

        # Look for the april tag
        if self.state == State.SEARCH_FOR_TAG:
            self.get_logger().info(f'State = SEARCH_FOR_TAG', once=True)

            # If april tag is within field of view, change state to move towards it
            if(self.april_flag):
                self.state = State.GO_TO_TAG
                # Align heading with tag TODO
            else:
                # Perform search algorithm
                self.search_for_tag()
                self.state = State.READ_TAG

        if self.state == State.GO_TO_TAG:
            self.get_logger().info(f'State = GO_TO_TAG')
            self.get_logger().info(f'self.april_x {self.april_x}')

            #self.set_relative_pose(self.april_z, self.april_y, self.april_x)
            # self.set_relative_pose(0.0, 0.0, self.april_x)
            self.set_relative_depth(self.april_x)
            # self.state = State.HOLD_POSE

        if self.state == State.HOLD_POSE:
            self.get_logger().info(f'State = HOLD_POSE', once=True)

        # Hold the robots depth and angle
        self.hold_pos(0.0, 0.0, 0.0)

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