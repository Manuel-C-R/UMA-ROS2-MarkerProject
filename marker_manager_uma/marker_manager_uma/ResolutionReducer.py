# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Reduce la imagen procesada a resolucion 480x480 y la publica
# en un topic
# Reduce the processed image to a resolution of 480x480 and publish it
# on a topic


import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ResolutionReducer(Node):

    def __init__(self):

        super().__init__("ResolutionReducer")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.image_topic_name = self.agent_name + "/processed_image"
        self.reduced_image_topic_name = self.agent_name + "/processed_image/reduced"

        self.subscription = self.create_subscription(Image, self.image_topic_name, self.video_receiver_callback, 10)
        self.subscription
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(Image, self.reduced_image_topic_name, 10)


    def video_receiver_callback(self, data):

        current_frame = self.bridge.imgmsg_to_cv2(data)

        # Reduce the image to 480x480
        resized_frame = cv.resize(current_frame,(480,480))

        # Publish msg
        msg = self.bridge.cv2_to_imgmsg(resized_frame, encoding="bgr8")
        self.publisher_.publish(msg)



def main(args=None):

    rclpy.init(args=args)

    resolution_reducer = ResolutionReducer()

    rclpy.spin(resolution_reducer)

    resolution_reducer.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
