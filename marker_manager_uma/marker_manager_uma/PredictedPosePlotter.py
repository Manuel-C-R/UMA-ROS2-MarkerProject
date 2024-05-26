# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Registra la localizacion estimada a partir de
# la mediana para cada uno de los marcadores y genera
# un mapa en el que aparecen todos (solo la ultima estimada
# para cada marcador)
# Register the estimated location based on the median 
# for each of the markers and generate a map showing all
# (only the latest estimated) locations for each marker.


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import gmplot
import datetime
import utm
import os


class PredictedPosePlotter(Node):

    def __init__(self):

        super().__init__("PredictedPosePlotter")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.predicted_pose_topic_name = self.agent_name + "/predicted_pose/F5L6"

        self.location_subscription = self.create_subscription(String, self.predicted_pose_topic_name,self.pose_receiver_callback, 10)
        self.location_subscription

        self.directory = "UMA-ROS2-DATA/" + self.agent_name +"/PredictedPoseMaps/"
        self.map_name = self.directory + self.agent_name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M") + ".html"

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.registeredID = []
        self.pose_data = []

    def pose_receiver_callback(self, data):

        full_string = data.data
        string_list = full_string.split(";")

        aruco_id = string_list[0]
        pose_list = string_list[1].split(",")

        X = float(pose_list[0])
        Y = float(pose_list[1])
        Z = float(pose_list[2])

        Number = int(pose_list[3])
        Letter = pose_list[4]

        latitude, longitude = utm.to_latlon(X,Y,Number,Letter)


        if aruco_id in self.registeredID:

            # ID registered
            index = self.registeredID.index(aruco_id)
            self.pose_data[index][0] = aruco_id
            self.pose_data[index][1] = X
            self.pose_data[index][2] = Y
            self.pose_data[index][3] = Z
            self.pose_data[index][4] = latitude
            self.pose_data[index][5] = longitude



        else:

            # ID not registered
            self.registeredID.append(aruco_id)
            self.pose_data.append([])
            index = self.registeredID.index(aruco_id)
            self.pose_data[index] = [aruco_id, X, Y, Z, latitude, longitude]


        # Map with predicted pose for all registered markers
        new_mapa = gmplot.GoogleMapPlotter(latitude,longitude,100)

        for marker in self.pose_data:
            new_mapa.scatter([marker[4]], [marker[5]], color='red',size=2, marker = True)
            location_info = f" ID: {marker[0]} | Latitude:{marker[4]}º | Longitude:{marker[5]}º | X: {marker[1]} m | Y: {marker[2]} m | Z: {marker[3]} m"
            new_mapa.text(marker[4] + 0.0001,marker[5] + 0.0001, text=location_info, color="red")

        new_mapa.map_type = "satellite"
        new_mapa.draw(self.map_name)


def main(args=None):

    rclpy.init(args=args)

    predicted_pose_plotter = PredictedPosePlotter()

    rclpy.spin(predicted_pose_plotter)

    predicted_pose_plotter.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
