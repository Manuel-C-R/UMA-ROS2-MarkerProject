from setuptools import find_packages, setup

package_name = 'marker_manager_uma'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manuel Cordoba Ramos',
    maintainer_email='manuelcordoba123@gmail.com',
    description='This package implements the processing related to UMA-ROS2-Android',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RawImageViewer = marker_manager_uma.RawImageViewer:main',
        	'DataProcessor = marker_manager_uma.DataProcessor:main',
        	'DataViewer = marker_manager_uma.DataViewer:main',
        	'UTMConverter = marker_manager_uma.UTMConverter:main',
        	'ProcessedImageViewer = marker_manager_uma.ProcessedImageViewer:main',
        	'AgentTracker = marker_manager_uma.AgentTracker:main',
        	'PoseCalculator = marker_manager_uma.PoseCalculator:main',
        	'PosePlotter = marker_manager_uma.PosePlotter:main',
        	'BatteryTracker = marker_manager_uma.BatteryTracker:main',
        	'DistanceCalculator = marker_manager_uma.DistanceCalculator:main',
        	'DistanceCollector = marker_manager_uma.DistanceCollector:main',
        	'PoseCollector = marker_manager_uma.PoseCollector:main',
        	'DataCollector = marker_manager_uma.DataCollector:main',
            'PosePredictor = marker_manager_uma.PosePredictor:main',
            'PredictedPoseCollector = marker_manager_uma.PredictedPoseCollector:main',
            'PredictedPosePlotter = marker_manager_uma.PredictedPosePlotter:main',
            'ResolutionReducer = marker_manager_uma.ResolutionReducer:main',
            'ReducedImageViewer = marker_manager_uma.ReducedImageViewer:main',
        ],
    },
)
