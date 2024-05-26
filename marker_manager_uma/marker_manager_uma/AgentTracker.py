# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Implementa el seguimiento del agente creando un mapa .html
# Implement agent tracking by creating an .html map


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import gmplot
import datetime
import os

class AgentTracker(Node):

    def __init__(self):

        super().__init__("AgentTracker")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.location_topic_name = self.agent_name + "/location"

        self.location_subscription = self.create_subscription(Float64MultiArray, self.location_topic_name,self.location_receiver_callback, 10)
        self.location_subscription

        self.directory = "UMA-ROS2-DATA/" + self.agent_name +"/AgentTracker/"

        self.filename = self.directory + self.agent_name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M") + ".html"

        if not os.path.exists(self.directory):
            os.makedirs(self.directory) 

        self.first = True

    def location_receiver_callback(self, data):

        latitude = data.data[0]
        longitude = data.data[1]

        if self.first:

            self.gmap = gmplot.GoogleMapPlotter(latitude,longitude,100) #latitude,longitude,zoom

            self.gmap.scatter([latitude], [longitude], color='red',size=2, marker = True)
            location_info = f"Latitude:{latitude}º | Longitude:{longitude}º"
            self.gmap.text(latitude + 0.0001,longitude + 0.0001, text=location_info, color="red")
            self.first = False

        else:

            self.gmap.scatter([latitude], [longitude], color='black',size=1, marker = False)

        self.gmap.map_type = "satellite"

        self.gmap.draw(self.filename)



        



def main(args=None):

    rclpy.init(args=args)

    agent_tracker = AgentTracker()

    rclpy.spin(agent_tracker)

    agent_tracker.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
