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
        self.Kp_x = 1.0
        self.Ki_x = 0.01

        # Set PI gains for angle
        self.Kp_a = 20.0
        self.Ki_a = 0.03

        # Set distance you want robot to go in front april tag
        self.dist_to_tag = 150 # About 6 inches
        
        # Create the timer
        self.timer = self.create_timer(0.005, self.timer_callback)

        # Make subscriber to detections topic to check if april tag is detected
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, "detections", self.detection_cb, 10)

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

        # self.update_april_pos()

        if self.state == State.INIT:
            self.get_logger().info(f'State = INIT', once=True)
            # Arm AUV
            self.arm()

            # Get initial depth reading
            self.get_depth()
          
            # Update state to look for apriltag
            self.state = State.READ_TAG
            # self.state = State.NOTHING

        if self.state == State.NOTHING:
            # if(x_diff == 0):
            #     diff_ang = 90.0
            # else:
            # diff_ang = self.wrap_angle(acos(target_y/x_diff)*57.2958)
                
            self.set_relative_pose(0.0, 0.0, 200.0, 30)
            self.move_xzr(0.0, 500.0, 0.0)
            while(1):
                self.get_logger().info(f'State = HOLD_POSE')
                self.set_relative_pose(0.0, 0.0, 0.0, 0.0)
                # self.hold_depth()
            self.state = 0


        if self.state == State.READ_TAG:
            self.get_logger().info(f'State = AGHHH', once=True)
            if(self.april_count > self.prev_april_count):
                self.state = State.SEARCH_FOR_TAG
            self.prev_april_count = self.april_count
            
        if self.state == State.SEARCH_FOR_TAG:
            self.get_logger().info(f'State = SEARCH_FOR_TAG', once=True)
            self.tmp_flag = self.april_flag
            if(self.tmp_flag):
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
            self.set_relative_pose(0.0, 0.0, 0.0)

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

    def set_relative_pose(self, x_diff, y_diff, z_diff, diff_ang):
        # POSES IN ROBOT FRAME
        target_depth = self.get_depth() + z_diff
        target_x = x_diff

        # # Calculate angle to target (convert to degrees)
        # if(x_diff == 0):
        #     diff_ang = 90.0
        # else:
        #     diff_ang = self.wrap_angle(acos(y_diff/x_diff)*57.2958)
        # target_angle = self.wrap_angle(self.get_heading() + diff_ang)

        # Keep moving robot until is within 1 cm of the target depth
        sum_error_z = 0
        sum_error_x = 0
        sum_error_ang = 0
        
        target_angle = self.get_heading() + diff_ang

        # Wrap angle - Mavlink uses angles from 0-359.99
        target_angle = self.wrap_angle(target_angle)

        sum_error_ang = 0

        target_depth = self.current_depth + z_diff

        # Keep moving robot until is within 1 cm of the target depth
        sum_error_depth = 0
        while(not isclose(target_depth, self.get_depth(), abs_tol = 10) and not isclose(target_angle, self.current_heading, abs_tol = 10)):

            error_depth = target_depth - self.current_depth
            self.get_logger().info(f'target depth {target_depth}')

            self.get_logger().info(f'depth error {error_depth}')

            sum_error_depth += error_depth
            if(sum_error_depth > 100):
                sum_error_depth = 100
            if(sum_error_depth < -100):
                sum_error_depth = -100

            throttle_depth = 500 + self.Kp_depth*(error_depth) + self.Ki_depth*(sum_error_depth)

            # Move robot
            
            #### ANGLE PART

            error_ang = self.wrap_angle(target_angle - self.get_heading())
            sum_error_ang += error_ang

            if(sum_error_ang > 20):
                sum_error_ang = 20
            if(sum_error_ang < -20):
                sum_error_ang = -20
            self.get_logger().info(f'angle target {target_angle}')

            self.get_logger().info(f'angle error {error_ang}')
            throttle_ang = self.Kp_a*(error_ang) + self.Ki_a*(sum_error_ang)
            # Move robot
            self.move_xzr(0.0, throttle_depth, throttle_ang)


    def detection_cb(self, msg):
        """
        """
        self.april_count += 1
        if(msg.detections):
            self.get_logger().info(f'CALBACK:!!!!!!!!!!!!!!!!!!!!!!!')
            self.april_flag = True
            self.update_april_pos()
        else:
            self.april_flag = False
            self.get_logger().info(f'CALBACK:????????????????????????')

    def search_for_tag(self):
        # This loop is blocking subscriber -> need to change so I can update tag pos
        self.get_logger().info(f'Search for tag')

        # Turn 10 Deg
        self.get_logger().info(f'Flag status: {self.april_flag}')
        
        # If you see the april tag -> change states
        if(self.tmp_flag):
            self.state = State.GO_TO_TAG
        else:
            # If not look around
            self.set_relative_angle(35)
            time.sleep(2)
            # Update april flag
            self.tmp_flag = self.april_flag
            
            # If you have turned almost a full circle, move up and try again
            self.angle_tracker += 35
            if(self.angle_tracker > 345):
                # Reset tracker
                self.angle_tracker = 0

                # Move up 10 cm
                self.set_relative_depth(200)

    def move_xzr(self, x_throttle, z_throttle, r_throttle):
        """
        """
        y_throttle = 0.0
        if(z_throttle > 1000):
            z_throttle = 1000
        
        if(z_throttle < 0):
            z_throttle = 0

        if(x_throttle > 1000):
            x_throttle = 1000
        
        if(x_throttle < -1000):
            x_throttle = -1000

        if(r_throttle > 1000):
            r_throttle = 1000

        if(r_throttle < -1000):
            r_throttle = -1000

        if(abs(r_throttle) < 250):
            if(r_throttle < 0):
                r_throttle = -250
            else:
                r_throttle = 250

        if(r_throttle < 0): # If throttle negative ->CCW
            x_throttle -= r_throttle
            y_throttle += r_throttle
        else:
            x_throttle += r_throttle
            y_throttle -= r_throttle

        self.master.mav.manual_control_send(
            self.master.target_system,
            int(x_throttle),
            int(y_throttle),
            int(z_throttle),
            0,
            0,
            0)

    def move_rotate(self, r_throttle):
        """
        """
        if(r_throttle > 1000):
            r_throttle = 1000

        if(r_throttle < -1000):
            r_throttle = -1000

        if(abs(r_throttle) < 250):
            if(r_throttle < 0):
                r_throttle = -250
            else:
                r_throttle = 250

        if(r_throttle < 0): # If throttle negative ->CCW
            x_throt = -r_throttle
            y_throt = r_throttle
        else:
            x_throt = r_throttle
            y_throt = -r_throttle

        self.master.mav.manual_control_send(
            self.master.target_system,
            int(x_throt),
            int(y_throt),
            500,
            0, #int(r_throttle),
            0,
            0)

    def set_relative_angle(self, diff):

        target_angle = self.get_heading() + diff

        # Wrap angle - Mavlink uses angles from 0-359.99
        target_angle = self.wrap_angle(target_angle)

        sum_error = 0
        while(not isclose(target_angle, self.current_heading, abs_tol = 1)):
            self.get_logger().info(f'Turning to angle', once=True)

            error = self.wrap_angle(target_angle - self.get_heading())
            sum_error += error

            if(sum_error > 20):
                sum_error = 20
            if(sum_error < -20):
                sum_error = -20

            throttle = self.Kp_a*(error) + self.Ki_a*(sum_error)
            # Move robot
            self.move_rotate(throttle)
        self.get_logger().info(f'Reached angle!')

    def wrap_angle(self, angle):
        if angle >= 0 and angle < 359.99:
            return angle
        elif angle < 0:
            wrapped_angle = angle % 359.99
            return 359.99 + wrapped_angle
        else:
            wrapped_angle = angle % 359.99
            return wrapped_angle

    def get_heading(self):
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
            if msg.get_type() == 'VFR_HUD':
                print("\n** Recieved Heading **")
                read_flag = False

        # Update current position everytime you read depth
        self.current_heading = self.wrap_angle(msg.to_dict()["heading"])
        return self.current_heading

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