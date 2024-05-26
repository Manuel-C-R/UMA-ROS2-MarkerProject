# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Calcula la localización georreferenciada en coordenadas UTM del marcador
# a partir de los valores de la IMU y modulo GNSS del smartphone
# y de la estimacion de su posicion respecto del smartphone
# Calculate the UTM georeferenced location of the marker
# based on IMU and GNSS module values from the smartphone
# and its estimated position relative to the smartphone


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class PoseCalculator(Node):

    def __init__(self):

        super().__init__("PoseCalculator")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.string_topic_name = self.agent_name + "/pose_data/F5L6"
        self.estimated_pose_topic_name = self.agent_name + "/estimated_pose/F5L6"

        self.subscription = self.create_subscription(String, self.string_topic_name,self.receiver_callback, 10)
        self.subscription

        self.publisher_ = self.create_publisher(String, self.estimated_pose_topic_name,10)


    def receiver_callback(self, data):

        full_string = data.data

        string_list = full_string.split(";")

        detection_id = string_list[0]

        aruco_id = string_list[1]

        marker_list = string_list[2].split(",")

        IMU_list = string_list[3].split(",")

        location_list = string_list[5].split(",")

        location_UTM_list = string_list[6].split(",")

        # Position of the fractal (F) relative to {C}
        P_F_C = np.array([[float(marker_list[5])*0.01],
                          [float(marker_list[6])*0.01],
                          [float(marker_list[7])*0.01],
                          [1]])
                        
        # Transformation matrix from {C} to {B}
        T_C_B = np.array([[0, -1,  0, 0],
                          [-1, 0,  0, 0],
                          [0,  0, -1, 0],
                          [0,  0,  0, 1]])
                        
        # Orientation quaternion of {B} relative to {A}
        q0 = float(IMU_list[0])
        q1 = float(IMU_list[1])
        q2 = float(IMU_list[2])
        q3 = float(IMU_list[3])


        # First row of the rotation matrix
        r00 = q0*q0 + q1*q1 - q2*q2 - q3*q3
        r01 = 2*(q1*q2 - q0*q3)
        r02 = 2*(q1*q3 + q0*q2)
     
        # Second row of the rotation matrix
        r10 = 2*(q1*q2 + q0*q3)
        r11 = q0*q0 - q1*q1 + q2*q2 - q3*q3
        r12 = 2*(q2*q3 - q0*q1)
     
        # Third row of the rotation matrix
        r20 = 2*(q1*q3 - q0*q2)
        r21 = 2*(q2*q3 + q0*q1)
        r22 = q0*q0 - q1*q1 - q2*q2 + q3*q3

        # Transformation matrix from {B} to {A}
        T_B_A = np.array([[r00, r01, r02,float(location_UTM_list[0])],
                         [r10, r11, r12, float(location_UTM_list[1])],
                         [r20, r21, r22, float(location_list[2])],
                         [0,   0,   0,   1]])

        # Position of the fractal (F) relative to {A}
        P_F_A = np.dot(np.dot(T_B_A,T_C_B),P_F_C)
        
        P_F_A_list = P_F_A.tolist()
        
        # XUTM, YUTM , Z , ZoneNumber, ZoneLetter
        P_F_A_str_list = [str(P_F_A_list[0][0]), str(P_F_A_list[1][0]), str(P_F_A_list[2][0]), location_UTM_list[3], location_UTM_list[4]]

        pose = ",".join(P_F_A_str_list)

        # Publish msg
        msg = String()
        msg_list = [detection_id,
                    aruco_id,
                    pose]
        
        msg.data = ";".join(msg_list)
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    pose_calculator = PoseCalculator()

    rclpy.spin(pose_calculator)

    pose_calculator.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
