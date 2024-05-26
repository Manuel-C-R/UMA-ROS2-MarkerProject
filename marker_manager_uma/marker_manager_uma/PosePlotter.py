# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# A partir de las localizaciones estimadas, crea un mapa único para
# cada una de ellas, en la que sale representada junto a sus datos.
# Además, crea un mapa para cada ID ArUco en el que aparecen todas 
# las localizaciones para ese marcador
# Create a unique map for each of the estimated locations, 
# where each location is represented along with its data.
# Additionally, create a map for each ArUco ID showing all 
# the locations for that marker.


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gmplot
import datetime
import utm
import os


class PosePlotter(Node):

    def __init__(self):

        super().__init__("PosePlotter")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.estimated_pose_topic_name = self.agent_name + "/estimated_pose/F5L6"

        self.location_subscription = self.create_subscription(String, self.estimated_pose_topic_name,self.pose_receiver_callback, 10)
        self.location_subscription

        self.directory = "UMA-ROS2-DATA/" + self.agent_name +"/EstimatedPoseMaps/"+ datetime.datetime.now().strftime("%Y-%m-%d-%H-%M") + "/"

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.registeredID = []
        self.maps = []
        self.maps_name = []


    def pose_receiver_callback(self, data):

        full_string = data.data
        string_list = full_string.split(";")

        detection_id = string_list[0]
        aruco_id = string_list[1]
        pose_list = string_list[2].split(",")

        X = float(pose_list[0])
        Y = float(pose_list[1])
        Z = float(pose_list[2])

        Number = int(pose_list[3])
        Letter = pose_list[4]

        latitude, longitude = utm.to_latlon(X,Y,Number,Letter)


        if aruco_id in self.registeredID:

            # ID registered
            index = self.registeredID.index(aruco_id)
            self.maps[index].scatter([latitude], [longitude], color='black',size=1, marker = False)
            self.maps[index].map_type = "satellite"
            self.maps[index].draw(self.maps_name[index])

        else:

            # ID not registered
            self.registeredID.append(aruco_id)
            self.maps.append(gmplot.GoogleMapPlotter(latitude,longitude,100))
            self.maps_name.append(self.directory + "ID-" + aruco_id + "-MAP.html")

            new_directory = self.directory + aruco_id
            os.makedirs(new_directory) 

            self.maps[-1].scatter([latitude], [longitude], color='red',size=2, marker = True)
            location_info = f" ID: {aruco_id} | Latitude:{latitude}º | Longitude:{longitude}º | X: {X} m | Y: {Y} m | Z: {Z} m"
            self.maps[-1].text(latitude + 0.0001,longitude + 0.0001, text=location_info, color="red")
            self.maps[-1].map_type = "satellite"
            self.maps[-1].draw(self.maps_name[-1])

        # Exclusive map of the location
        new_mapa_name = self.directory + aruco_id + "/ID-" + aruco_id + "-D-" + detection_id + ".html"
        new_mapa = gmplot.GoogleMapPlotter(latitude,longitude,100)
        new_mapa.scatter([latitude], [longitude], color='red',size=2, marker = True)
        location_info = f" ID: {aruco_id} | ID detection: {detection_id} | Latitude:{latitude}º | Longitude:{longitude}º | X: {X} m | Y: {Y} m | Z: {Z} m"
        new_mapa.text(latitude + 0.0001,longitude + 0.0001, text=location_info, color="red")
        new_mapa.map_type = "satellite"
        new_mapa.draw(new_mapa_name)


def main(args=None):

    rclpy.init(args=args)

    pose_plotter = PosePlotter()

    rclpy.spin(pose_plotter)

    pose_plotter.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
