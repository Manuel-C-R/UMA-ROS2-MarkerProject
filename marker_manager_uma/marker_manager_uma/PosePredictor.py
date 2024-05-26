# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Almacena todas las localizaciones estimadas para un marcador,
# calcula la mediana y publica el resultado en un topic.
# Store all the estimated locations for a marker,
# calculate the median, and publish the result to a topic.


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np


class PosePredictor(Node):

    def __init__(self):

        super().__init__("PosePredictor")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.estimated_pose_topic_name = self.agent_name + "/estimated_pose/F5L6"
        self.predicted_pose_topic_name = self.agent_name + "/predicted_pose/F5L6"

        self.location_subscription = self.create_subscription(String, self.estimated_pose_topic_name,self.pose_receiver_callback, 10)
        self.location_subscription

        self.publisher_ = self.create_publisher(String, self.predicted_pose_topic_name,10)


        self.registeredID = []
        self.XData = []
        self.YData = []
        self.ZData = []


    def pose_receiver_callback(self, data):

        full_string = data.data
        string_list = full_string.split(";")

        aruco_id = string_list[1]
        pose_list = string_list[2].split(",")

        X = float(pose_list[0])
        Y = float(pose_list[1])
        Z = float(pose_list[2])

        Number = int(pose_list[3])
        Letter = pose_list[4]


        if aruco_id in self.registeredID:

            # ID registered
            index = self.registeredID.index(aruco_id)
            self.XData[index].append(X)
            self.YData[index].append(Y)
            self.ZData[index].append(Z)

            medianX = np.median(self.XData[index])
            medianY = np.median(self.YData[index])
            medianZ = np.median(self.ZData[index])

            # Publish msg
            msg = String()

            estimated_pose_list = [str(medianX), str(medianY), str(medianZ), str(Number), Letter]

            estimated_pose_list_msg = ",".join(estimated_pose_list)

            msg_list = [aruco_id,
                        estimated_pose_list_msg,
                        str(len(self.XData[index]))]
        
            msg.data = ";".join(msg_list)
            self.publisher_.publish(msg)



        else:

            # ID not registered
            self.registeredID.append(aruco_id)
            self.XData.append([])
            self.YData.append([])
            self.ZData.append([])

            self.XData[-1].append(X)
            self.YData[-1].append(Y)
            self.ZData[-1].append(Z)

            medianX = np.median(self.XData[-1])
            medianY = np.median(self.YData[-1])
            medianZ = np.median(self.ZData[-1])

            # Publish msg
            msg = String()

            estimated_pose_list = [str(medianX), str(medianY), str(medianZ), str(Number), Letter]

            estimated_pose_list_msg = ",".join(estimated_pose_list)

            msg_list = [aruco_id,
                        estimated_pose_list_msg,
                        str(len(self.XData[-1]))]
        
            msg.data = ";".join(msg_list)
            self.publisher_.publish(msg)
            
            
def main(args=None):

    rclpy.init(args=args)

    pose_predictor = PosePredictor()

    rclpy.spin(pose_predictor)

    pose_predictor.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
