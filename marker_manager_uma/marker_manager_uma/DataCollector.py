# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Almacena los mensajes de un topic en formato json
# Store the messages from a topic in JSON format


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import datetime
import os

class DataCollector(Node):

    def __init__(self):

        super().__init__("DataCollector")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.string_topic_name = self.agent_name + "/pose_data/F5L6"

        self.subscription = self.create_subscription(String, self.string_topic_name,self.receiver_callback, 10)
        self.subscription

        self.directory = "UMA-ROS2-DATA/" + self.agent_name +"/PoseData/"


        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.jsonfile = []
        self.jsonfile_name = self.directory + self.agent_name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M") + ".json"


    def receiver_callback(self, data):

        full_string = data.data

        string_list = full_string.split(";")

        detection_id = string_list[0]

        aruco_id = string_list[1]

        marker_list = string_list[2].split(",")

        IMU_list = string_list[3].split(",")

        battery = string_list[4]

        location_list = string_list[5].split(",")

        location_UTM_list = string_list[6].split(",")

        new_data = {
            'AgentName' : self.agent_name,
            'Detection-ID': int(detection_id),
            'Resolution' : f"{marker_list[11]}x{marker_list[12]}",
            'Camera' : self.getCamera(marker_list[13]),
            'Aruco-ID': str(aruco_id),
            'DetectedSubmarkers': f"{marker_list[0]}-{marker_list[1]}-{marker_list[2]}-{marker_list[3]}-{marker_list[4]}",
            'x': float(marker_list[5]),
            'y': float(marker_list[6]),
            'z': float(marker_list[7]),
            'a': float(marker_list[8]),
            'b': float(marker_list[9]),
            'g': float(marker_list[10]),
            'Battery': float(battery),
            'Ow': float(IMU_list[0]),
            'Ox': float(IMU_list[1]),
            'Oy': float(IMU_list[2]),
            'Oz': float(IMU_list[3]),
            'Gx': float(IMU_list[4]),
            'Gy': float(IMU_list[5]),
            'Gz': float(IMU_list[6]),
            'Ax': float(IMU_list[7]),
            'Ay': float(IMU_list[8]),
            'Az': float(IMU_list[9]),
            'Lat': float(location_list[0]),
            'Long': float(location_list[1]),
            'Alt': float(location_list[2]),
            'X_UTM': float(location_UTM_list[0]),
            'Y_UTM': float(location_UTM_list[1]),
            'Zone': f"{location_UTM_list[3]}{location_UTM_list[4]}",
        }

        self.jsonfile.append(new_data)


        with open(self.jsonfile_name, 'w') as archivo:
            json.dump(self.jsonfile, archivo)

    def getCamera(self,id_cam):

        if id_cam == "1":
            return "Wide-Angle"
        else:
            return "Normal"    

def main(args=None):

    rclpy.init(args=args)

    data_collector = DataCollector()

    rclpy.spin(data_collector)

    data_collector.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
