# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Obtiene las coordenadas UTM a partir de los valores de latitud,
# longitud y altitud, para publicarlas en un topic
# Obtain UTM coordinates from latitude, longitude, and altitude values,
# and publish them on a topic


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import numpy as np
import utm


class UTMConverter(Node):

    def __init__(self):

        super().__init__("UTMConverter")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.location_topic_name = self.agent_name + "/location"
        self.utm_location_topic_name = self.agent_name + "/location/UTM"

        self.location_subscription = self.create_subscription(Float64MultiArray, self.location_topic_name,self.location_receiver_callback, 10)
        self.location_subscription

        self.publisher_ = self.create_publisher(String, self.utm_location_topic_name,10)


    def location_receiver_callback(self, data):

        latitude = data.data[0]
        longitude = data.data[1]
        altitude = data.data[2]

        utm_coords = utm.from_latlon(latitude, longitude)

        x_utm, y_utm, number, letter = utm_coords

        msg_list = [str(x_utm),str(y_utm),str(altitude),str(number),str(letter)]

        msg = String()
        msg.data = ",".join(msg_list)
        
        # Publish msg
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    utm_converter = UTMConverter()

    rclpy.spin(utm_converter)

    utm_converter.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
