# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Almacena los mensajes de un topic en formato json
# Store the messages from a topic in JSON format


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import datetime
import os

class DistanceCollector(Node):

    def __init__(self):

        super().__init__("DistanceCollector")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.string_topic_name = self.agent_name + "/distance/F5L6"

        self.subscription = self.create_subscription(String, self.string_topic_name,self.receiver_callback, 10)
        self.subscription

        self.directory = "UMA-ROS2-DATA/" + self.agent_name +"/DistanceData/"+ datetime.datetime.now().strftime("%Y-%m-%d-%H-%M") + "/"

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.registeredID = []
        self.jsonfiles = []
        self.jsonfiles_name = []


    def receiver_callback(self, data):

        full_string = data.data

        string_list = full_string.split(";")

        detection_id = string_list[0]

        aruco_id = string_list[1]

        distance = string_list[2]

        new_data = {
            'Detection-ID': int(detection_id),
            'Aruco-ID': aruco_id,
            'Distance': float(distance)

        }


        if aruco_id in self.registeredID:

            # ID registered
            index = self.registeredID.index(aruco_id)
            self.jsonfiles[index].append(new_data)
            ruta_archivo = self.jsonfiles_name[index]

            with open(ruta_archivo, 'w') as archivo:
                json.dump(self.jsonfiles[index], archivo)

        else:
            # ID not registered
            self.registeredID.append(aruco_id)
            self.jsonfiles.append([])
            self.jsonfiles_name.append(self.directory + "ID-" + aruco_id + ".json")

            self.jsonfiles[-1].append(new_data)
            ruta_archivo = self.jsonfiles_name[-1]
            
            with open(ruta_archivo, 'w') as archivo:
                json.dump(self.jsonfiles[-1], archivo)



def main(args=None):

    rclpy.init(args=args)

    distance_collector = DistanceCollector()

    rclpy.spin(distance_collector)

    distance_collector.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
