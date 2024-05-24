// Author: Manuel Cordoba Ramos <manuelcordoba123@gmail.com>
// Implementa la deteccion de marcadores fractales y la estimacion de su pose
// Contiene los archivos de calibracion de Google Pixel 7 Pro
// Implement fractal marker detection and pose estimation.
// Contains calibration files for Google Pixel 7 Pro.


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp" 
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <aruco/aruco.h>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

class GP7FractalProcessor : public rclcpp::Node {

private:

	//Parametros de entrada
	// Input parameters.
	int resolution_;
    std::string agentname_;
	float size_;

	// Variables usadas en el codigo
	// Variables used in the code.
    cv::Mat frame;
    aruco::FractalDetector FDetector;
    aruco::CameraParameters my_cam;
	std::string myFolderPath;
	int image_width, image_height;

	// Nombre de los topics
	// Topic names.
	std::string camera_topic_name;
	std::string processed_image_topic_name;
	std::string marker_data_topic_name;
	std::string update_camera_topic_name;

public:
    GP7FractalProcessor() : Node("GP7FractalProcessor") {
    
    	//Declaro parametros de entrada
		// Declare input parameters.

		// Resolucion elegida, por defecto 0
		// Chosen resolution, default to 0
		/*
		0 - 800x600
		1 - 1280x720
		2 - 1280x960
		3 - 1920x1080
		4 - 2048x1536
		5 - 3840x2160
		*/

    	this->declare_parameter<int>("resolution", 0);
    	this->get_parameter("resolution", resolution_);

		// Nombre de agente elegido, por defecto PX7
		// Chosen agent name, default to PX7
    	this->declare_parameter<std::string>("AgentName", "PX7");
    	this->get_parameter("AgentName", agentname_);

		// Tamaño del fractal en [m], por defecto 0.197
		// Fractal size in [m], default to 0.197.
    	this->declare_parameter<float>("size", 0.197);
    	this->get_parameter("size", size_);

		camera_topic_name = agentname_ + "/camera";
		processed_image_topic_name = agentname_ + "/processed_image";
		marker_data_topic_name = agentname_ + "/detected_fractal/F5L6";
		update_camera_topic_name = agentname_ + "/setcamera";

		

        //Suscripcion de la imagen del movil
		// Subscribe to the smartphone image.
        subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(camera_topic_name, rclcpp::SensorDataQoS(), std::bind(&GP7FractalProcessor::topic_callback, this, std::placeholders::_1));
        

		//Suscripcion al actualizador
		// Subscribe to the updater.
		subscription_update = this->create_subscription<std_msgs::msg::UInt8MultiArray>(update_camera_topic_name, 10, std::bind(&GP7FractalProcessor::update_callback, this, std::placeholders::_1));


        //Publicador de imagen procesada
		// Publisher of processed image.
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(processed_image_topic_name,10);
        FDetector.setConfiguration("FRACTAL_5L_6");

		//Publicador de los valores del fractal procesado
		// Publisher of processed fractal values.
    	publisher_v = this->create_publisher<std_msgs::msg::Float32MultiArray>(marker_data_topic_name,10);
        
        //Creo la ruta relativa a la carpeta de las calibraciones
		// Create the relative path to the calibration folder.
        std::string packagePath = ament_index_cpp::get_package_share_directory("fractal_pkg");
        myFolderPath = packagePath + "/calibration";

		// Cargo el archivo de calibracion elegido
		// Load the chosen calibration file.
		my_cam.readFromXMLFile(myFolderPath + getYMLcalibration(resolution_));
		
		//Muestro un mensaje por pantalla indicando el tamaño del fractal
		// Display a message indicating the size of the fractal.
		std::string info_size_msg = "Fractal marker size: " + std::to_string(size_) + " m.";
		RCLCPP_INFO(this->get_logger(), info_size_msg.c_str());


		// Muestro un mensaje por pantalla
		// Display a message on the screen.
		RCLCPP_INFO(this->get_logger(), getInfoMessage(resolution_).c_str());
        
		// Parametros del detector de fractales
		// Parameters of the fractal detector.
    	FDetector.setParams(my_cam,size_);
    	
    }

private:
    void topic_callback(const std_msgs::msg::UInt8MultiArray & msg ) {

        std::vector<unsigned char> buf(msg.data.begin(), msg.data.end());

		// Decodificar la imagen usando OpenCV
		// Decode the image using OpenCV.
		frame = cv::imdecode(buf, cv::IMREAD_COLOR);

		

		if(FDetector.detect(frame)){ // Detecta el fractal // Detect the fractal.



            FDetector.drawMarkers(frame); // Dibuja el fractal // Draw the fractal.

            
            
            if(FDetector.poseEstimation()){ // Estima la pose // Estimate the pose

            	FDetector.draw3d(frame); // Dibuja el cubo 3D // Draw the 3D cube.

            	cv::Mat rvec = FDetector.getRvec(); // Vector de rotacion // Rotation vector.
            	cv::Mat tvec = FDetector.getTvec(); // Vector de translacion // Translation vector.
            	
            	auto marcadores = FDetector.getMarkers();
				//Devuelve la longitud del vector, por lo tanto conozco el numero de submarkers que ha detectado.
				// Returns the length of the vector, thus I know the number of submarkers it has detected.
            	int n_markers = marcadores.size(); 
            	
            	std::string text_id="";

				// Por defecto vector a 0, ningun submarker detectado
				// Default to vector at 0, no submarkers detected.
				std::vector<int> detectedSubMarkers(5,0); 

            	for (int j=0;j<n_markers;j++){
					// Pongo a 1 el submarker que ha detectado
					// Set the detected submarker to 1.
					detectedSubMarkers[marcadores[j].id] = 1; 
            	}

				for (int j=0;j<5;j++){
					//Paso a string el vector de SubMarkers detectados
					// Convert the detected SubMarkers vector to a string.
					text_id += std::to_string(detectedSubMarkers[j]);
				}




            	float X = 100*tvec.at<double>(0, 0);
            	float Y = 100*tvec.at<double>(1, 0);
            	float Z = 100*tvec.at<double>(2, 0);
            	
            	float a = rvec.at<double>(0, 0);
            	float b = rvec.at<double>(1, 0);
            	float g = rvec.at<double>(2, 0);
            	
            	std::string textoX = "X: " + std::to_string(X) + " cm";
            	std::string textoY = "Y: " + std::to_string(Y) + " cm";
				std::string textoZ = "Z: " + std::to_string(Z) + " cm";

				cv::Point posicionX(10, 30);
				cv::Point posicionY(10, 60);
				cv::Point posicionZ(10, 90);
				cv::Point posicionid(10, 120);
	
				// Configurar el tipo de letra, tamaño, color y grosor del texto
				// Configure font type, size, color, and thickness of the text.
    			int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    			double fontScale = 1.0;
    			cv::Scalar color_red(0, 0, 255); // BGR
				cv::Scalar color_green(0, 255, 0); // BGR
				cv::Scalar color_blue(255, 0, 0); // BGR
				cv::Scalar color_white(255, 255, 255); //BGR
    			int thickness = 1;
    			int lineType = cv::LINE_AA;
    		
    			cv::putText(frame, textoX, posicionX, fontFace, fontScale, color_red, thickness, lineType);
    			cv::putText(frame, textoY, posicionY, fontFace, fontScale, color_green, thickness, lineType);
    			cv::putText(frame, textoZ, posicionZ, fontFace, fontScale, color_blue, thickness, lineType);
    			cv::putText(frame, text_id, posicionid, fontFace, fontScale, color_white, thickness, lineType);
    		
    			// Dimensiones de la imagen publicada
				// Dimensions of the published image.
				cv::Size dimImage = frame.size();
				image_width = dimImage.width;
				image_height = dimImage.height;

    			auto msg_valores = std::make_shared<std_msgs::msg::Float32MultiArray>();
    			msg_valores->data ={static_cast<float>(detectedSubMarkers[0]), 
									static_cast<float>(detectedSubMarkers[1]), 
									static_cast<float>(detectedSubMarkers[2]), 
									static_cast<float>(detectedSubMarkers[3]),
									static_cast<float>(detectedSubMarkers[4]),
									X, Y, Z,
									a, b, g,
									static_cast<float>(image_width),
									static_cast<float>(image_height),
									static_cast<float>(0)};

				// Publico el mensaje
				// Publish the message.

    			publisher_v->publish(*msg_valores);
    		      	
        }
        }
		// Publico la imagen
		// Publish the image.
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg_.get());

    }



	void update_callback(const std_msgs::msg::UInt8MultiArray & update_msg ) {
		

		if ((update_msg.data.size() == 3) && (update_msg.data[0] <= 5) && (update_msg.data[1] <= 1) && (update_msg.data[2] == 111)) {
			
			// Mensaje valido
			// Valid message.


				resolution_ = update_msg.data[0];
				
				// Cargo el archivo de calibracion elegido
				// Load the chosen calibration file.
				my_cam.readFromXMLFile(myFolderPath + getYMLcalibration(resolution_));
		
				// Muestro un mensaje por pantalla
				// Display a message on the screen.
				RCLCPP_INFO(this->get_logger(), getInfoMessage(resolution_).c_str());
        
				// Parametros del detector de fractales
				// Parameters of the fractal detector.
    			FDetector.setParams(my_cam,size_); 
		
		}


	}

    std_msgs::msg::Float32MultiArray::SharedPtr msg_valores; //
    sensor_msgs::msg::Image::SharedPtr msg_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_update;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_v; //


	std::string getInfoMessage(int resolution){

		std::string info_msg = "Pixel 7 - ";


		info_msg += "Normal - ";


		// Resolucion
		// Resolution
		if (resolution == 1){
			info_msg += "1280x720";
		} 
		else if (resolution == 2){
			info_msg += "1280x960";
		}
		else if (resolution == 3){
			info_msg += "1920x1080";
		}
		else if (resolution == 4){
			info_msg += "2048x1536";
		}
		else if (resolution == 5){
			info_msg += "3840x2160";
		}
		else{
			info_msg += "800x600";
		}

		return info_msg;
	}

	std::string getYMLcalibration(int resolution){

		std::string calibration_file_name = "/pixel7_";
		
		// Resolucion
		// Resolution
		if (resolution == 1){
			calibration_file_name += "1280x720.yml";
		} 
		else if (resolution == 2){
			calibration_file_name += "1280x960.yml";
		}
		else if (resolution == 3){
			calibration_file_name += "1920x1080.yml";
		}
		else if (resolution == 4){
			calibration_file_name += "2048x1536.yml";
		}
		else if (resolution == 5){
			calibration_file_name += "3840x2160.yml";
		}
		else{
			calibration_file_name += "800x600.yml";
		}

		return calibration_file_name;
	}
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GP7FractalProcessor>());
    rclcpp::shutdown();
    return 0;
}
