# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Calcula la distancia a partir de un vector de traslación y la publica en un topic
# Calculate distance from a translation vector and publish it on a topic


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DistanceCalculator(Node):

    def __init__(self):

        super().__init__("DistanceCalculator")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.string_topic_name = self.agent_name + "/pose_data/F5L6"
        self.distance_topic_name = self.agent_name + "/distance/F5L6"

        self.subscription = self.create_subscription(String, self.string_topic_name,self.receiver_callback, 10)
        self.subscription

        self.publisher_ = self.create_publisher(String, self.distance_topic_name,10)


    def receiver_callback(self, data):

        full_string = data.data

        string_list = full_string.split(";")

        detection_id = string_list[0]

        aruco_id = string_list[1]

        marker_list = string_list[2].split(",")


        x = float(marker_list[5])
        y = float(marker_list[6])
        z = float(marker_list[7])

        dist = (x*x + y*y + z*z)**(0.5)

        # Publish msg
        msg = String()
        msg_list = [detection_id,
                    aruco_id,
                    str(dist)]
        
        msg.data = ";".join(msg_list)
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    distance_calculator = DistanceCalculator()

    rclpy.spin(distance_calculator)

    distance_calculator.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
