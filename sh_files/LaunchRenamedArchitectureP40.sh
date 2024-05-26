#!/bin/bash

AgentName="$1"

Resolution="$2"

Camera="$3"

Size="$4"

#RawImageViewer
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_RawImageViewer node with Agent name: $AgentName';
ros2 run marker_manager_uma RawImageViewer --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_RawImageViewer;
exec bash"

#HP40FractalProcessor
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_HP40FractalProcessor node with Agent name: $AgentName';
ros2 run marker_detector_uma HP40FractalProcessor --ros-args -p AgentName:=$AgentName -p camera:=$Camera -p resolution:=$Resolution -p size:=$Size --remap __node:=${AgentName}_HP40FractalProcessor;
exec bash"

#ArucoProcessor
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_ArucoProcessor node with Agent name: $AgentName';
ros2 run marker_detector_uma ArucoProcessor --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_ArucoProcessor;
exec bash"

#DataProcessor
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_DataProcessor node with Agent name: $AgentName';
ros2 run marker_manager_uma DataProcessor --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_DataProcessor;
exec bash"

#ProcessedImageViewer
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_ProcessedImageViewer node with Agent name: $AgentName';
ros2 run marker_manager_uma ProcessedImageViewer --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_ProcessedImageViewer;
exec bash"


#UTMConverter
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_UTMConverter node with Agent name: $AgentName';
ros2 run marker_manager_uma UTMConverter --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_UTMConverter;
exec bash"

#DataViewer
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_DataViewer node with Agent name: $AgentName';
ros2 run marker_manager_uma DataViewer --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_DataViewer;
exec bash"

#DataCollector
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_DataCollector node with Agent name: $AgentName';
ros2 run marker_manager_uma DataCollector --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_DataCollector;
exec bash"


#BatteryTracker
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_BatteryTracker node with Agent name: $AgentName';
ros2 run marker_manager_uma BatteryTracker --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_BatteryTracker;
exec bash"

#AgentTracker
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_AgentTracker node with Agent name: $AgentName';
ros2 run marker_manager_uma AgentTracker --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_AgentTracker;
exec bash"

#DistanceCalculator
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_DistanceCalculator node with Agent name: $AgentName';
ros2 run marker_manager_uma DistanceCalculator --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_DistanceCalculator;
exec bash"

#DistanceCollector
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_DistanceCollector node with Agent name: $AgentName';
ros2 run marker_manager_uma DistanceCollector --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_DistanceCollector;
exec bash"

#PoseCalculator
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_PoseCalculator node with Agent name: $AgentName';
ros2 run marker_manager_uma PoseCalculator --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_PoseCalculator;
exec bash"


#PosePlotter
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_PosePlotter node with Agent name: $AgentName';
ros2 run marker_manager_uma PosePlotter --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_PosePlotter;
exec bash"

#PoseCollector
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_PoseCollector node with Agent name: $AgentName';
ros2 run marker_manager_uma PoseCollector --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_PoseCollector;
exec bash"

#PosePredictor
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_PosePredictor node with Agent name: $AgentName';
ros2 run marker_manager_uma PosePredictor --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_PosePredictor;
exec bash"

#PredictedPoseCollector
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_PredictedPoseCollector node with Agent name: $AgentName';
ros2 run marker_manager_uma PredictedPoseCollector --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_PredictedPoseCollector;
exec bash"

#PredictedPosePlotter
terminator -e "source /opt/ros/humble/setup.bash;
source ros2_ws/install/local_setup.bash;
echo 'Launching ${AgentName}_PredictedPosePlotter node with Agent name: $AgentName';
ros2 run marker_manager_uma PredictedPosePlotter --ros-args -p AgentName:=$AgentName --remap __node:=${AgentName}_PredictedPosePlotter;
exec bash"

