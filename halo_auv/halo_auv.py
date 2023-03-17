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
from halo_auv_interfaces.srv import AuvPose
import math 

class State(Enum):
    """States to keep track of where the system is."""
    SEARCH_FOR_TAG = auto(),
    INIT = auto(),
    MATCH_DEPTH = auto(),
    GO_TO_TAG = auto(),
    HOLD_POSE = auto(),
    READ_TAG = auto(),
    NOTHING = auto()




class HaloAuv(Node):
    """
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('auv_node')

        # ===== Set up Mavlink Comms ===== #
        # Create the connection to the top-side computer as companion computer/autopilot
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

        # Wait a heartbeat before sending commands
        # self.master.wait_heartbeat()

        # # Initialize mode 
        # self.set_mode()

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
        
        # Initialize service client
        self.auv_pose_client = self.create_client(AuvPose, 'set_relative_pos')
        self.auv_abs_pose_client = self.create_client(AuvPose, 'set_absolute_pos')

        # while not self.auv_pose_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.auv_pose_req = AuvPose.Request()

        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'detections',
            self.detection_cb,
            10)

        # Create the timer
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):

        # Arm robot before moving on
        if self.state == State.INIT:
            self.get_logger().info(f'State = INIT', once=True)
            # Arm AUV
            # self.arm()
            self.get_logger().info(f'bobobobo')

            self.send_auv_pose_request(0.0, 100.0, 0.0)
            self.get_logger().info(f'AYAGAGAG')

            # Update state to look for apriltag
            self.state = State.READ_TAG

        # Only switch states once you have checked if you are seeing an april tag or not
        if self.state == State.READ_TAG:
            self.get_logger().info(f'State = READ_TAG', once=True)
            # Have a counter in april tag callback, check that you have entered this callback
            # before searching for the tag
            if(self.april_count > self.prev_april_count):
                self.state = State.SEARCH_FOR_TAG
            self.prev_april_count = self.april_count

        # Look for the april tag
        if self.state == State.SEARCH_FOR_TAG:
            self.get_logger().info(f'State = SEARCH_FOR_TAG', once=True)

            # If april tag is within field of view, change state to move towards it
            if(self.april_flag):
                self.state = State.GO_TO_TAG
            else:
                # Perform search algorithm
                self.search_for_tag()
                # Check if you see tag
                self.state = State.READ_TAG

        if self.state == State.GO_TO_TAG:
            self.get_logger().info(f'State = GO_TO_TAG {self.april_x}')
            theta = math.atan2(self.april_y, self.april_x)
            # self.send_auv_pose_request(self.april_z, self.april_x, theta)
            self.send_abs_auv_pose_request(0.0, self.get_depth()-self.april_x, 0.0)
            # while(self.april_count < self.prev_april_count):
            #     print('wait for tag')
            # self.prev_april_count = self.april_count
            self.update_april_pos()
            self.state = State.NOTHING

        if self.state == State.HOLD_POSE:
            self.get_logger().info(f'State = HOLD_POSE', once=True)

        # Hold the robots depth and angle
        # self.hold_pos(0.0, 0.0, 0.0)

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
                read_flag = False

        # Update current position everytime you read depth
        self.current_depth = msg.to_dict()["relative_alt"]

        return self.current_depth

    def update_april_pos(self):
        # Listen to transformation from apriltag
        try:
            tf_2_tag = self.tf_buffer.lookup_transform(
                'camera_link',
                'tag0',
                rclpy.time.Time())
            self.april_x = 1000*tf_2_tag.transform.translation.x
            self.april_y = 1000*tf_2_tag.transform.translation.y
            self.april_z = 1000*tf_2_tag.transform.translation.z
        except:
            pass

    def send_auv_pose_request(self, x, depth, heading):
        self.auv_pose_req.x = x
        self.auv_pose_req.depth = depth
        self.auv_pose_req.heading = heading

        self.future = self.auv_pose_client.call_async(self.auv_pose_req)
        # rclpy.spin_until_future_complete(self, self.future)
        # self.get_logger().info(f'HOW DID I GET HERE')

        return self.future.result()

    def send_abs_auv_pose_request(self, x, depth, heading):
        self.auv_pose_req.x = x
        self.auv_pose_req.depth = depth
        self.auv_pose_req.heading = heading

        self.future = self.auv_abs_pose_client.call_async(self.auv_pose_req)
        # rclpy.spin_until_future_complete(self, self.future)
        # self.get_logger().info(f'HOW DID I GET HERE')

        return self.future.result()

    def detection_cb(self, msg):
        """
        """
        self.april_count += 1
        if(msg.detections):
            self.april_flag = True
            self.update_april_pos()
        else:
            self.april_flag = False

    def search_for_tag(self):
        # To look for tag, turn a defined angle 
        self.send_auv_pose_request(0.0, 0.0, 1.0)

        # If you have turned almost a full circle, move up and try again
        self.angle_tracker += 1
        self.get_logger().info(f'Angle tracker {self.angle_tracker}')

        if(self.angle_tracker > 345):
            # Reset tracker

            self.angle_tracker = 0

            # Move up 10 cm
            self.send_auv_pose_request(0.0, 100.0, 0.0)

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
        self.get_logger().info(f'ARMED!')

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

    def restart_pixhawk(self):
        self.master.reboot_autopilot()

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    halo_auv_node = HaloAuv()

    # Spin the node so the callback function is called.
    rclpy.spin(halo_auv_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    halo_auv_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()