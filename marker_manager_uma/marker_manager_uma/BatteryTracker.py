# Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
# Implementa el seguimiento de la batería del agente creando una gráfica .pdf
# Implement battery tracking for the agent by creating a .pdf graph


import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import numpy as np
import time
import os
import datetime
import matplotlib.pyplot as plt

class BatteryTracker(Node):

    def __init__(self):

        super().__init__("BatteryTracker")
        self.declare_parameter("AgentName","P40")
        self.agent_name= self.get_parameter("AgentName").get_parameter_value().string_value

        self.battery_topic_name = self.agent_name + "/battery"

        self.battery_subscription = self.create_subscription(UInt8, self.battery_topic_name,self.battery_receiver_callback, 10)
        self.battery_subscription

        self.directory = "UMA-ROS2-DATA/" + self.agent_name +"/BatteryTracker/"

        self.filename = self.directory + self.agent_name + "-" + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M") + ".pdf"

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.first_time = time.time()
        self.y_ticks = np.arange(0, 101, 5) 
        self.x = []
        self.y = []

        self.fig, self.ax = plt.subplots()

        self.first = True

        
    def battery_receiver_callback(self, data):

        battery = data.data
        current_time = time.time()
        self.y.append(battery)
        self.x.append(current_time - self.first_time)
        
        self.ax.clear()
        self.ax.plot(self.x, self.y, linewidth=2, color = "blue")

        self.ax.set_yticks(self.y_ticks)
        self.ax.grid(True)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Battery (%)")
        self.ax.set_title(f"{self.agent_name} battery evolution")
        self.ax.set_ylim(0, 105)

        
        self.fig.savefig(self.filename)
            
    

def main(args=None):

    rclpy.init(args=args)

    battery_tracker = BatteryTracker()

    rclpy.spin(battery_tracker)

    battery_tracker.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
