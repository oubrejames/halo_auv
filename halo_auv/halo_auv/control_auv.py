import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from math import isclose
from halo_auv_interfaces.srv import AuvPose
import math

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

        self.arm()

        # Save depth and have a class variable to keep track of depth
        self.current_depth = self.get_depth()

        # Save heading and have a class variable to keep track of heading (degrees)
        self.current_heading = self.get_heading()

        # Initialize target positions for depth and heading
        self.target_depth = self.current_depth
        self.target_heading = self.current_heading
        self.target_x = 0

        # Set PI gains for depth
        self.Kp_depth = 2.0
        self.Ki_depth = 0.01

        # Set PI gains for angle
        self.Kp_a = 20.0
        self.Ki_a = 0.03

        # Set PI gains for x
        self.Kp_x = 1.0
        self.Ki_x = 0.01

        # Initialize integral error
        self.heading_err_sum = 0.0
        self.depth_err_sum = 0.0

        # Create a service to move change target postion values
        self.set_pose = self.create_service(AuvPose, "set_relative_pos", self.set_pose_cb)
        self.set_pose_abs = self.create_service(AuvPose, "set_absolute_pose", self.set_abs_pose_cb)


        # Create a 200 hz timer
        self.timer_cnt = 0
        self.timer = self.create_timer(0.005, self.timer_callback)


    def timer_callback(self):
        self.timer_cnt += 1

        # self.get_logger().info(f'Target heading {self.target_heading}')
        # self.get_logger().info(f'Current heading {self.current_heading}')

        # Calculate state error
        depth_err = self.target_depth - self.get_depth()
        self.target_heading = self.wrap_angle(self.target_heading)
        heading_err = self.target_heading - self.get_heading()
        # self.get_logger().info(f' Error heading {heading_err}')
        x_error = self.target_x - 0.15 # 0.15 is a buffer so you dont hit wall

        # Calculate integral error
        self.depth_err_sum += depth_err
        if(self.depth_err_sum > 100):
            self.depth_err_sum = 100
        if(self.depth_err_sum < -100):
            self.depth_err_sum = -100

        self.heading_err_sum += heading_err
        if(self.heading_err_sum > 20):
            self.heading_err_sum = 20
        if(self.heading_err_sum < -20):
            self.heading_err_sum = -20
        
        #  If error is sufficently small, reset integral error
        if(isclose(depth_err, 0.0, abs_tol = 10)):
            self.depth_err_sum = 0.0
            
        if(isclose(heading_err, 0.0, abs_tol = 3)):
            self.heading_err_sum = 0.0

        # Calculate verticle throttle
        depth_throttle = 500 + self.Kp_depth*depth_err + self.Ki_depth*self.depth_err_sum

        # Caclulate rotational throttle
        rot_throttle = self.Kp_a*heading_err + self.Ki_a*self.heading_err_sum

        # Run x controller at slower frequency because ... TODO
        x_throttle = self.Kp_x*x_error

        # From rotational throttle, edit x and y throttle to achieve rotation -> shouldnt have to do this
        self.send_cmd(x_throttle, depth_throttle, rot_throttle)
        
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
                read_flag = False

        # Update current position everytime you read depth
        self.current_heading = self.wrap_angle(msg.to_dict()["heading"])
        return self.current_heading

    def send_cmd(self, x_throttle, z_throttle, r_throttle):
        y_throttle = 0.0

        # Bound z throttle
        if(isclose(self.get_depth(), 0.0, abs_tol=30)):
            z_throttle = 500

        if(z_throttle > 1000):
            z_throttle = 1000
        if(z_throttle < 0):
            z_throttle = 0

        # Bound x throttle
        if(x_throttle > 1000):
            x_throttle = 1000
        if(x_throttle < -1000):
            x_throttle = -1000

        # Bound rotation throttle
        if(r_throttle > 1000):
            r_throttle = 1000
        if(r_throttle < -1000):
            r_throttle = -1000

        if(r_throttle < 0): # If throttle negative ->CCW
            x_throttle += r_throttle
            y_throttle -= r_throttle
            # x_throttle -= r_throttle
            # y_throttle += r_throttle
        else:
            x_throttle += r_throttle
            y_throttle -= r_throttle

        # Send commmand to robot
        self.master.mav.manual_control_send(
            self.master.target_system,
            int(x_throttle),
            int(y_throttle),
            int(z_throttle),
            0,
            0)

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

    def set_pose_cb(self, request, response):
        self.target_depth += request.depth
        self.target_heading += request.heading
        self.target_x += request.x
        return response

    def set_abs_pose_cb(self, request, response):
        self.target_depth = request.depth
        self.target_heading = request.heading
        self.target_x = request.x
        return response

    def wrap_angle(self, angle):
        wrapped_angle = math.fmod(angle, 360.0)
        if wrapped_angle < 0:
            wrapped_angle += 360.0
        return wrapped_angle

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