# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Muestra la imagen reducida
# Display reduced image


import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ReducedImageViewer(Node):

    def __init__(self):

        super().__init__("ReducedImageViewer")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.image_topic_name = self.agent_name + "/processed_image/reduced"

        self.subscription = self.create_subscription(Image, self.image_topic_name, self.video_receiver_callback, 10)
        self.subscription

        self.bridge = CvBridge()


    def video_receiver_callback(self, data):

        current_frame = self.bridge.imgmsg_to_cv2(data)
        self.get_logger().info(f"Showing reduced video from {self.image_topic_name} : {current_frame.shape[1]}x{current_frame.shape[0]}")
        cv.namedWindow(f"Reduced image from {self.agent_name}", cv.WINDOW_NORMAL);        
        cv.imshow(f"Reduced image from {self.agent_name}", current_frame)
        cv.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    reduced_image_viewer = ReducedImageViewer()

    rclpy.spin(reduced_image_viewer)

    reduced_image_viewer.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
