import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import gi
import numpy as np
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import yaml
from sensor_msgs.msg import CameraInfo
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_path

class CameraPublisher(Node):
    """
    Create an CameraPublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('auv_camera')
            
        # Get camera parameters
        # self.declare_parameter("auv_cam_yaml", "/home/oubre/auv_ws/src/halo_auv/config/auv_cam_params.yaml", ParameterDescriptor(
        #     description="Path to camera parameter yaml file"))
        # self.camera_params = self.get_parameter(
        #     "auv_cam_yaml").get_parameter_value().string_value

        # Path for the calibration file
        self.package_path = get_package_share_path('halo_auv')
        self.camera_params = str(self.package_path) + '/auv_cam_params.yaml'

        # Create camera info message
        self.camera_info_msg = self.yaml_to_CameraInfo(self.camera_params)
        
        # Create camera info publisher
        self.pub_cam_info = self.create_publisher(CameraInfo, "camera_info", 10)

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
            
        # We will publish a message every 0.1 seconds
        timer_period = 0.2  # seconds
            
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
                
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.

        # # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # From bluerov code
        Gst.init(None)

        self.port = 5600
        self.latest_frame = self._new_frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    # Function modified from 
    # https://github.com/udacity/CarND-Capstone/blob/master/ros/src/camera_info_publisher/yaml_to_camera_info_publisher.py
    def yaml_to_CameraInfo(self, calib_yaml):
        """
        Parse a yaml file containing camera calibration data (as produced by
        rosrun camera_calibration cameracalibrator.py) into a
        sensor_msgs/CameraInfo msg.
        Parameters
        ----------
        yaml_fname : str
            Path to yaml file containing camera calibration data
        Returns
        -------
        camera_info_msg : sensor_msgs.msg.CameraInfo
            A sensor_msgs.msg.CameraInfo message containing the camera calibration
            data
        """
        # Load data from file
        # calib_data = yaml.safe_load(calib_yaml)
        with open(calib_yaml, 'r') as file:
            calib_data = yaml.safe_load(file)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = "map"
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.k = calib_data["camera_matrix"]["data"]
        camera_info_msg.d = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.r = calib_data["rectification_matrix"]["data"]
        camera_info_msg.p = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """


        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.

        if self.frame_available():
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            frame = self.frame()
            img_msg_frame = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg_frame.header.frame_id = "camera_link"
            self.camera_info_msg.header.stamp = img_msg_frame.header.stamp

            self.publisher_.publish(img_msg_frame)
            # Publish camera info
            self.pub_cam_info.publish(self.camera_info_msg)

            # IN camera info add a sequence number and look if they are matching

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None


    def run(self):
        """ Get frame to update _new_frame
        """
        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)


    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = CameraPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()