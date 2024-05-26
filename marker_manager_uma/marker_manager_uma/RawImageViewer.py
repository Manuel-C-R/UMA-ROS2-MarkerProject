# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Muestra la imagen sin procesar, obtenida del smartphone
# Display raw image from the smartphone


import cv2 as cv
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class RawImageViewer(Node):

    def __init__(self):

        super().__init__("RawImageViewer")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.topic_name = self.agent_name + "/camera"

        self.subscription = self.create_subscription(UInt8MultiArray, self.topic_name,self.video_receiver_callback, qos_profile_sensor_data)
        self.subscription

        self.bridge = CvBridge()


    def video_receiver_callback(self, data):

        buf = np.asarray(data.data, dtype=np.uint8)
        current_frame = cv.imdecode(buf,cv.IMREAD_COLOR)
        
        if current_frame is not None and current_frame.shape[0] > 0 and current_frame.shape[1] > 0:

            text = f"Showing raw video from {self.topic_name}, resolution: {current_frame.shape[1]}x{current_frame.shape[0]}"
            self.get_logger().info(text)
            cv.namedWindow(f"Raw image from {self.agent_name}", cv.WINDOW_NORMAL)
            cv.imshow(f"Raw image from {self.agent_name}", current_frame)
            cv.waitKey(1)

        else:

            self.get_logger().info("Empty image")
       

def main(args=None):

    rclpy.init(args=args)

    raw_image_viewer = RawImageViewer()

    rclpy.spin(raw_image_viewer)

    raw_image_viewer.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
