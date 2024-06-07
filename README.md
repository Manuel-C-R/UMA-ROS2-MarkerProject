# UMA-ROS2-MarkerProject


Este repositorio contiene una aplicación para Android, llamada UMA-ROS2-Android (UR2A), y una versión lite de esta, junto a una arquitectura desarrollada en ROS 2 que implementa la detección de marcadores fractales y ArUco.

UR2A es capaz de comunicarse con una arquitectura de ROS 2 (haciendo uso de ROS2-Java), transmitiendo la imagen del smartphone e información sensorial utilizando sus sensores internos, además puede recibir comandos externos que modifiquen su comportamiento, y visualizar información relacionada con el procesamiento realizado en la arquitectura en la que se encuentra.

La arquitectura desarrollada puede procesar imágenes que contengan marcadores de referencia fractales y ArUco, consiguiendo estimar su pose relativa al smartphone y georreferenciarla en coordenadas UTM.

El funcionamiento ha sido validado a través de pruebas indoor y outdoor haciendo uso de un UAV.

**Una vez presentado el proyecto para el que se ha desarrollado este software, se actualizará el repositorio con información sobre su uso y los resultados obtenidos.**



This repository contains an Android application called UMA-ROS2-Android (UR2A) and a lite version of it, along with an architecture developed in ROS 2 that implements the detection of fractal and ArUco markers.

UR2A can communicate with a ROS 2 architecture (using ROS2-Java), transmitting the smartphone's image and sensor information using its internal sensors. It can also receive external commands that modify its behavior and display information related to the processing done within the architecture it is part of.

The developed architecture can process images containing fractal and ArUco fiducial markers, estimating their pose relative to the smartphone and georeferencing them in UTM coordinates.

The functionality has been validated through indoor and outdoor tests using a UAV.

**Once the project for which this software has been developed is presented, the repository will be updated with information on its usage and the results obtained.**
