#!/bin/bash

AgentName="$1"

Resolution="$2"

Camera="$3"

Size="$4"

#RawImageViewer
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching RawImageViewer node with Agent name: $AgentName';
ros2 run marker_manager_uma RawImageViewer --ros-args -p AgentName:=$AgentName;
exec bash"

#HP40FractalProcessor
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching HP40FractalProcessor node with Agent name: $AgentName';
ros2 run marker_detector_uma HP40FractalProcessor --ros-args -p AgentName:=$AgentName -p camera:=$Camera -p resolution:=$Resolution -p size:=$Size;
exec bash"


#ArucoProcessor
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ArucoProcessor node with Agent name: $AgentName';
ros2 run marker_detector_uma ArucoProcessor --ros-args -p AgentName:=$AgentName;
exec bash"

#DataProcessor
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching DataProcessor node with Agent name: $AgentName';
ros2 run marker_manager_uma DataProcessor --ros-args -p AgentName:=$AgentName;
exec bash"

#ProcessedImageViewer
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ProcessedImageViewer node with Agent name: $AgentName';
ros2 run marker_manager_uma ProcessedImageViewer --ros-args -p AgentName:=$AgentName;
exec bash"


#UTMConverter
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching UTMConverter node with Agent name: $AgentName';
ros2 run marker_manager_uma UTMConverter --ros-args -p AgentName:=$AgentName;
exec bash"

#DataViewer
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching DataViewer node with Agent name: $AgentName';
ros2 run marker_manager_uma DataViewer --ros-args -p AgentName:=$AgentName;
exec bash"

#DataCollector
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching DataCollector node with Agent name: $AgentName';
ros2 run marker_manager_uma DataCollector --ros-args -p AgentName:=$AgentName;
exec bash"


#BatteryTracker
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching BatteryTracker node with Agent name: $AgentName';
ros2 run marker_manager_uma BatteryTracker --ros-args -p AgentName:=$AgentName;
exec bash"

#AgentTracker
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching AgentTracker node with Agent name: $AgentName';
ros2 run marker_manager_uma AgentTracker --ros-args -p AgentName:=$AgentName;
exec bash"

#DistanceCalculator
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching DistanceCalculator node with Agent name: $AgentName';
ros2 run marker_manager_uma DistanceCalculator --ros-args -p AgentName:=$AgentName;
exec bash"


#DistanceCollector
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching DistanceCollector node with Agent name: $AgentName';
ros2 run marker_manager_uma DistanceCollector --ros-args -p AgentName:=$AgentName;
exec bash"

#PoseCalculator
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching PoseCalculator node with Agent name: $AgentName';
ros2 run marker_manager_uma PoseCalculator --ros-args -p AgentName:=$AgentName;
exec bash"


#PosePlotter
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching PosePlotter node with Agent name: $AgentName';
ros2 run marker_manager_uma PosePlotter --ros-args -p AgentName:=$AgentName;
exec bash"

#PoseCollector
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching PoseCollector node with Agent name: $AgentName';
ros2 run marker_manager_uma PoseCollector --ros-args -p AgentName:=$AgentName;
exec bash"

#PosePredictor
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching PosePredictor node with Agent name: $AgentName';
ros2 run marker_manager_uma PosePredictor --ros-args -p AgentName:=$AgentName;
exec bash"

#PredictedPoseCollector
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching PredictedPoseCollector node with Agent name: $AgentName';
ros2 run marker_manager_uma PredictedPoseCollector --ros-args -p AgentName:=$AgentName;
exec bash"

#PredictedPosePlotter
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching PredictedPosePlotter node with Agent name: $AgentName';
ros2 run marker_manager_uma PredictedPosePlotter --ros-args -p AgentName:=$AgentName;
exec bash"
