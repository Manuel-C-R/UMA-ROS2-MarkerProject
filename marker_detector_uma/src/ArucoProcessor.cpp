// Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
// Implementa la deteccion de marcadores ArUco
// Implement ArUco marker detection


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp" 
#include "std_msgs/msg/u_int8_multi_array.hpp" 
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include "marker_detector_uma/aruco_nano.h"
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class ArucoProcessor : public rclcpp::Node {

private:
    cv::Mat frame;
    std::string agentname_;
	std::string camera_topic_name;
	std::string aruco_topic_name;
	int id;

public:
    ArucoProcessor() : Node("ArucoProcessor") {
    
    	//Parametros de entrada
		//Input parameters

		// Nombre de agente elegido, por defecto P40
		//Chosen agent name, default to P40
    	this->declare_parameter<std::string>("AgentName", "P40");
    	this->get_parameter("AgentName", agentname_);

		camera_topic_name = agentname_ + "/camera";
		aruco_topic_name = agentname_ + "/detected_aruco/ID";


        //Suscripcion de la imagen del movil
		// Subscription to smartphone image.
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(camera_topic_name, rclcpp::SensorDataQoS(), std::bind(&ArucoProcessor::topic_callback, this, std::placeholders::_1));
        

		//Publicador de la id del aruco detectado
		// Publisher of the detected ArUco ID.
    	publisher_ = this->create_publisher<std_msgs::msg::String>(aruco_topic_name,10);
        
    }

private:
    void topic_callback(const std_msgs::msg::UInt8MultiArray & msg ) {

		// Decodificar la imagen usando OpenCV
		// Decode the image using OpenCV.
        std::vector<unsigned char> buf(msg.data.begin(), msg.data.end());
		frame = cv::imdecode(buf, cv::IMREAD_COLOR);


		// Detecta el ArUco
		// Detect the ArUco.
		auto markers = aruconano::MarkerDetector::detect(frame); 

		if (markers.size() > 0){

			id = markers[0].id;
		}
		else {

			// Valor por efecto si no se detecta
			// Default value if not detected.
			id = -1; 
		}

		// Creo el mensaje y lo publico
		// Create the message and publish it.
		auto id_msg= std::make_shared<std_msgs::msg::String>();
    	id_msg->data = std::to_string(id);
    	publisher_->publish(*id_msg);
	}

    std_msgs::msg::String::SharedPtr id_msg; //
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;	
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoProcessor>());
    rclcpp::shutdown();
    return 0;
}
