# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Reune la información obtenida y la publica en un topic
# Gather the obtained information and publish it to a topic


import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import numpy as np


class DataProcessor(Node):

    def __init__(self):

        super().__init__("DataProcessor")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        # Default values
        self.last_battery_string = "NaN"
        self.last_location_string = "NaN,"*2 + "NaN"
        self.last_location_UTM_string = "NaN,"*4 + "NaN"
        self.last_IMU_string = "NaN,"*9 + "NaN"
        self.last_marker_string = "NaN,"*10 + "NaN"
        self.last_aruco_nano_string = "NaN"

        # Topic's name
        self.battery_topic_name = self.agent_name + "/battery"
        self.location_topic_name = self.agent_name + "/location"
        self.location_UTM_topic_name = self.agent_name + "/location/UTM"
        self.IMU_topic_name = self.agent_name + "/IMU"
        self.marker_topic_name = self.agent_name + "/detected_fractal/F5L6"
        self.string_topic_name = self.agent_name + "/pose_data/F5L6"
        self.aruco_nano_topic_name = self.agent_name + "/detected_aruco/ID"
        self.aruco_nano_reset_topic_name = self.agent_name + "/reset_aruco/ID"

        # Detection counter
        self.detection_id = 0


        # Subscriptions
        self.battery_subscription = self.create_subscription(UInt8, self.battery_topic_name,self.battery_receiver_callback, 10)
        self.battery_subscription

        self.location_subscription = self.create_subscription(Float64MultiArray, self.location_topic_name,self.location_receiver_callback, 10)
        self.location_subscription

        self.location_UTM_subscription = self.create_subscription(String, self.location_UTM_topic_name,self.location_UTM_receiver_callback, 10)
        self.location_UTM_subscription

        self.IMU_subscription = self.create_subscription(Float32MultiArray, self.IMU_topic_name,self.IMU_receiver_callback, 10)
        self.IMU_subscription

        self.marker_subscription = self.create_subscription(Float32MultiArray, self.marker_topic_name,self.marker_receiver_callback, 10)
        self.marker_subscription

        self.aruco_nano_subscription = self.create_subscription(String, self.aruco_nano_topic_name,self.aruco_nano_receiver_callback, 10)
        self.aruco_nano_subscription
        
        self.reset_aruco_nano_subscription = self.create_subscription(UInt8, self.aruco_nano_reset_topic_name,self.reset_aruco_nano_receiver_callback, 10)
        self.reset_aruco_nano_subscription


        # Publisher
        self.publisher_ = self.create_publisher(String, self.string_topic_name,10)


    def battery_receiver_callback(self, data):

        last_battery = data.data
        self.last_battery_string = str(last_battery)


    def location_receiver_callback(self, data):

        last_location = np.array(data.data, dtype=np.float64)

        location_list = last_location.tolist()
        location_list_string = [str(elemento) for elemento in location_list]

        self.last_location_string = ','.join(location_list_string)


    def location_UTM_receiver_callback(self, data):
        self.last_location_UTM_string = data.data


    def IMU_receiver_callback(self, data):

        last_IMU = np.array(data.data, dtype=np.float32)

        IMU_list = last_IMU.tolist()
        IMU_list_string = [str(elemento) for elemento in IMU_list]

        self.last_IMU_string = ','.join(IMU_list_string)


    def reset_aruco_nano_receiver_callback(self, data):

        if (data.data == 111): #Password
            self.last_aruco_nano_string = "NaN"
            self.get_logger().info("Aruco ID has been reset!")


    def aruco_nano_receiver_callback(self, data):

        if (data.data != "-1"):
            self.last_aruco_nano_string = data.data


    def marker_receiver_callback(self, data):

        self.detection_id += 1
        
        last_marker = np.array(data.data, dtype=np.float32)

        marker_list = last_marker.tolist()
        marker_list_string = [str(int(elemento)) for elemento in marker_list[:5]]
        marker_list_string += [str(elemento) for elemento in marker_list[5:11]]
        marker_list_string += [str(int(elemento)) for elemento in marker_list[11:]]
        self.last_marker_string = ",".join(marker_list_string)

        # Publish msg
        msg = String()
        msg_list = [str(self.detection_id),
                    self.last_aruco_nano_string,
                    self.last_marker_string,
                    self.last_IMU_string,
                    self.last_battery_string,
                    self.last_location_string,
                    self.last_location_UTM_string]
        
        msg.data = ";".join(msg_list)
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    data_processor = DataProcessor()

    rclpy.spin(data_processor)

    data_processor.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
