# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Muestra el mensaje publicado en un topic
# Display the message published on a topic.


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataViewer(Node):

    def __init__(self):

        super().__init__("DataViewer")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.string_topic_name = self.agent_name + "/pose_data/F5L6"

        self.subscription = self.create_subscription(String, self.string_topic_name,self.receiver_callback, 10)
        self.subscription


    def receiver_callback(self, data):

        full_string = data.data

        string_list = full_string.split(";")

        detection_id = string_list[0]

        aruco_nano = string_list[1]

        marker_list = string_list[2].split(",")

        IMU_list = string_list[3].split(",")

        battery = string_list[4]

        location_list = string_list[5].split(",")

        location_UTM_list = string_list[6].split(",")

        msg = "\n" + "*"*40 + "\n"

        msg += "Agent name:" + self.agent_name + "\n\n"

        msg += f"Detection ID: {detection_id}\n\n"

        msg += "Image Data:\n"

        msg += f"Resolution: {marker_list[11]}x{marker_list[12]}\n"

        msg += f"Used camera: {self.getCamera(marker_list[13])}\n\n"

        msg += "Aruco ID:" + aruco_nano + "\n\n"

        msg += "Fractal Data:\n"

        msg += f"Detected submarkers :  {marker_list[0]}-{marker_list[1]}-{marker_list[2]}-{marker_list[3]}-{marker_list[4]}\n"

        msg += f"x: {marker_list[5]} cm  \ny: {marker_list[6]} cm  \nz: {marker_list[7]} cm\n"

        msg += f"a: {marker_list[8]} \nb: {marker_list[9]} \ng: {marker_list[10]} \n\n"

        msg += "Battery: " + battery + " %\n\n"

        msg += "IMU: \n"

        msg += f"Orientation: \nw: {IMU_list[0]} \nx: {IMU_list[1]} \ny: {IMU_list[2]} \nz: {IMU_list[3]}\n" 

        msg += f"Gyroscope: \nx: {IMU_list[4]} rad/s \ny: {IMU_list[5]} rad/s \nz: {IMU_list[6]} rad/s\n"

        msg += f"Accelerometer: \nx: {IMU_list[7]} m/s² \ny: {IMU_list[8]} m/s² \nz: {IMU_list[9]} m/s²\n\n" 

        msg += "Location: \n"

        msg += f"Latitude: {location_list[0]} º\n"
        msg += f"Longitude: {location_list[1]} º\n"
        msg += f"Altitude: {location_list[2]} m\n\n"

        msg += "UTM:\n"
        msg += f"X: {location_UTM_list[0]} m\n"
        msg += f"Y: {location_UTM_list[1]} m\n"
        msg += f"Zone: {location_UTM_list[3]}{location_UTM_list[4]}\n"

        msg += "*"*40


        self.get_logger().info(msg)


    def getCamera(self,id_cam):

        if id_cam == "1":
            return "Wide-Angle"
        else:
            return "Normal"    




def main(args=None):

    rclpy.init(args=args)

    data_viewer = DataViewer()

    rclpy.spin(data_viewer)

    data_viewer.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
