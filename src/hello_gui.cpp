#include "hello_gui.h"
#include "ui_hello_gui.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QVBoxLayout>
#include <QApplication>
#include <QtWebEngineWidgets/QWebEngineView>
#include <QString>
#include <sstream>
#include <regex>
#include <QDateTime>
#include <QFile>
#include <geometry_msgs/PointStamped.h>
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QScrollBar>
#include <opencv2/imgcodecs.hpp> // Include for cv::imwrite
#include <QTimer> // Include QTimer for periodic saving
#include <opencv2/opencv.hpp>
#include <QMessageBox> // Include QMessageBox for displaying messages
#include <filesystem> // Include filesystem library for path checking
#include <QTimer>
#include <QApplication>
#include <QUrl>
#include <sstream>
#include <string>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QVBoxLayout>
#include <QApplication>
#include <QtWebEngineWidgets/QWebEngineView>
#include <QString>
#include <sstream>
#include <regex>
#include <QDateTime>
#include <QWebEnginePage>
#include <QWebEngineSettings>
#include <geometry_msgs/PointStamped.h>
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QScrollBar>
#include<iostream>
#include <QInputDialog>
#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <opencv2/opencv.hpp>
//#include <QVBoxLayout>
//#include <QDir>
//#include <QDateTime>
//#include <QDebug>
//#include <QGuiApplication>
//#include <QProcess>

double pitchValue = 0.0;
double rollValue = 0.0;
double yawValue = 0.0;

static double previousPitchValue = 0.0;
static double previousRollValue = 0.0;
static double previousYawValue = 0.0;

float globalLatitude2 = 0.0;
float globalLongitude2 = 0.0;
float yawValueMap2 = 0.0;

cv::VideoCapture videoCapture2;




HelloGui::HelloGui(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::HelloGui)
{
    ui->setupUi(this);
    nh_ = ros::NodeHandle("~");
    ros_timer = new QTimer(this);

    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    ros_timer->start(100);
    
    {
    QFile file("/code_drive/src/kamikaze_main/pj_tflite_track_deepsort/gui_data/txt_files/velocity.txt"); // Specify your desired file name and path
    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        file.close(); // Simply open in truncate mode to clear content
        std::cout<<"cleared velocity.txt"<<std::endl;
    }
    }
    {
    QFile file("/code_drive/src/kamikaze_main/pj_tflite_track_deepsort/gui_data/txt_files/target_id.txt"); // Specify your desired file name and path
    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        file.close(); // Simply open in truncate mode to clear content
        std::cout<<"cleared target_id.txt"<<std::endl;
    }
    }
    
    QTimer *dateTimeTimer = new QTimer(this);
    connect(dateTimeTimer, &QTimer::timeout, this, &HelloGui::updateDateTimeDisplay);
    dateTimeTimer->start(1000); // Update every second
    
    
    
    video_pathss = {"/code_drive/src/6573939-sd_640_360_24fps.mp4",
"/code_drive/src/6573939-sd_640_360_24fps.mp4",
 "/code_drive/src/6573939-sd_640_360_24fps.mp4",
 "/code_drive/src/6573939-sd_640_360_24fps.mp4"};
    labelss_ = {ui->label, ui->label_2, ui->label_3, ui->label_4};

    // Start video feed threads
    startVideoThreadss();

    // Create a web engine view and set it up
    webEngineView_ = new QWebEngineView(this);
    
    webEngineView_->setAttribute(Qt::WA_TranslucentBackground);
    webEngineView_->setStyleSheet("background:transparent;");
    webEngineView_->page()->setBackgroundColor(Qt::transparent);

    QVBoxLayout *layout_map = new QVBoxLayout(ui->webEngineViewPlaceholder);
    layout_map->addWidget(webEngineView_);
    
    
    webEngineView_map = new QWebEngineView(this);
    QVBoxLayout *layout = new QVBoxLayout(ui->webEngineViewPlaceholder_2);
    layout->addWidget(webEngineView_map);
    
    renderHTML("/code_drive/src/template_gui_package/map_hud.html");
    



    // Add subscription to battery topic
    battery_sub_ = nh_.subscribe("/drone/battery", 1, &HelloGui::batteryCallback, this);
    flight_mode_sub_ = nh_.subscribe("/drone/flight_mode", 1, &HelloGui::flightmodeCallback, this);
    position_sub_ = nh_.subscribe("/drone/position", 1, &HelloGui::PositionmodeCallback, this);
    speed_sub_ = nh_.subscribe("/drone/velocity", 1, &HelloGui::speedCallback, this);
    euler_sub_ = nh_.subscribe("/drone/euler", 1, &HelloGui::eulerCallback, this);
    home_position_sub_ = nh_.subscribe("/drone/home_position", 1, &HelloGui::homePositionCallback, this);
    gps_info_sub_ = nh_.subscribe("/drone/gps_info", 1, &HelloGui::gpsInfoCallback, this);
    image_sub_ = nh_.subscribe("/webcam/image", 1, &HelloGui::displayImage, this);
    
    connect(ui->pushButton_3, &QPushButton::clicked, this, &HelloGui::saveToFile);
    connect(ui->pushButton_5, &QPushButton::clicked, this, &HelloGui::clearFile);

    
    QIcon icon("/home/kamikaze/Downloads/3d-video-record-icon-png.webp"); // Specify the correct path to your image
    ui->pushButton_2->setStyleSheet("QPushButton { text-align: left; padding-right: 50px; }"
                                 "QPushButton:pressed { padding-right: 48px; }"); // Adj
    ui->pushButton_2->setIcon(icon);
    
    renderHTMLMap("/code_drive/src/template_gui_package/mainwindow_map.html");
    
    QTimer *fileUpdateTimer = new QTimer(this);
connect(fileUpdateTimer, &QTimer::timeout, this, &HelloGui::updatePolygon);
fileUpdateTimer->start(5000); // Check for updates every 5 seconds
    
    QObject::connect(webEngineView_map, &QWebEngineView::loadFinished, [this](bool success) {
    if (success) {
    
    	//QString rotationAngle = QString::number(yawValueMap);
        // JavaScript to manage markers
        QString jsManageMarkers = R"(
        var rotationAngle = 0;
    	var lastMarker = null;
        var polyline = L.polyline([], {color: 'red'}).addTo(map_3387e5a10ff0da1a0bd5b3b27c4260fc); // Ensure this matches the map variable in your HTML/JS
    	
    	var imagePath = '/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/media/drone1.png';
    	
    	var isDragging = false;

        // Update isDragging flag on drag start
        map_3387e5a10ff0da1a0bd5b3b27c4260fc.on('dragstart', function() {
        isDragging = true;
        });

        // Reset isDragging flag on drag end
        map_3387e5a10ff0da1a0bd5b3b27c4260fc.on('dragend', function() {
        isDragging = false;
        });
        
        var markers = [];


map_3387e5a10ff0da1a0bd5b3b27c4260fc.on('contextmenu', function(event) {
    // Format the latitude and longitude values to 7 decimal places
    var lat = event.latlng.lat.toFixed(7);
    var lng = event.latlng.lng.toFixed(7);

    // Create a new marker at the clicked location
    var newMarker = L.marker(event.latlng)
        .addTo(map_3387e5a10ff0da1a0bd5b3b27c4260fc);

    // Attach a popup to the marker that behaves like a label
    newMarker.bindPopup(lat + ", " + lng, { closeButton: false, autoClose: false })
        .openPopup();
        
        

    // Prevent the popup from closing when the map is clicked
    newMarker.getPopup().options.closeOnClick = false;

    // Listen for the popup close event to remove the marker (if you decide to allow closing the popup)
    newMarker.on('popupclose', function() {
        map_3387e5a10ff0da1a0bd5b3b27c4260fc.removeLayer(newMarker);
    });
});


    	
       function addAndRemoveMarker(lat, lng, rotationAngle) {
    
       var customIcon = L.divIcon({
                className: 'custom-div-icon',
                html: "<img src='" + imagePath + "' style='transform: rotate(" + rotationAngle + "deg);' width='128' height='128'/>",
                iconSize: [32, 32],
                iconAnchor: [64, 64]
            });
            
        if (lastMarker != null) {
            map_3387e5a10ff0da1a0bd5b3b27c4260fc.removeLayer(lastMarker);
        }
        lastMarker = L.marker([lat, lng], {icon: customIcon}).addTo(map_3387e5a10ff0da1a0bd5b3b27c4260fc);
        polyline.addLatLng([lat, lng]); // Add new point to polyline
        
        if (!isDragging ) {
        map_3387e5a10ff0da1a0bd5b3b27c4260fc.setView([lat, lng], map_3387e5a10ff0da1a0bd5b3b27c4260fc.getZoom());
    }
        }
        
        )";

        // Add the JavaScript function to the page
        webEngineView_map->page()->runJavaScript(jsManageMarkers);


        // Timer to control the addition and removal of markers
        QTimer *timer = new QTimer(this);
        QObject::connect(timer, &QTimer::timeout, [this, globalLatitude2, globalLongitude2]() mutable {
            if (globalLatitude2 != 0.0 && globalLongitude2 != 0.0){
		    QString jsCode = QString::asprintf("addAndRemoveMarker(%f, %f, %f);", globalLatitude2, globalLongitude2, yawValueMap2);
		    webEngineView_map->page()->runJavaScript(jsCode);
            }
            
            //std::cout << "addAndRemoveMarker(" << globalLatitude2 << ", " << globalLongitude2 << ")" << std::endl;


        });

        // Start the timer to add and remove a marker every 0.5 seconds
        timer->start(5);
    }
});
    
}


HelloGui::~HelloGui()
{
    delete ui;
    delete ros_timer;
    delete updateContentTimer;
}

void HelloGui::spinOnce()
{
    if (!ros::ok()) {
        QApplication::quit();
    }
    ros::spinOnce();
}

void HelloGui::saveToFile() {
    QFile file("/code_drive/src/kamikaze_main/pj_tflite_track_deepsort/gui_data/txt_files/target_id.txt");
    if (file.open(QIODevice::WriteOnly)) {
        QTextStream out(&file);
        // Set textEdit to be editable
        ui->textEdit->setReadOnly(false); // This line ensures the textEdit is editable
        QString textValue = ui->textEdit->toPlainText();
        int value = textValue.toInt(); // Convert the string to integer, assuming it's an integer you want to write
        out << value << "\n"; // Writing the integer value followed by a newline
        file.close();
        //std::cout << "Data written to file successfully." << std::endl;
    }
}
// Slot to clear the content of the text file
void HelloGui::clearFile() {
    QFile file("/code_drive/src/kamikaze_main/pj_tflite_track_deepsort/gui_data/txt_files/target_id.txt"); // Specify your desired file name and path
    if (file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        file.close(); // Simply open in truncate mode to clear content
        //std::cout<<"cleared...................................................."<<std::endl;
    }
}

void HelloGui::displayImage(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        // Convert ROS sensor_msgs/Image to OpenCV image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        // Convert OpenCV Mat to Qt QImage
        QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        qimg = qimg.rgbSwapped(); // OpenCV uses BGR, QImage uses RGB, so we need to swap channels

        // Resize the image to fit the label
        QPixmap pixmap = QPixmap::fromImage(qimg).scaled(ui->label_11->size(), Qt::KeepAspectRatio);

        // Display the image in the QLabel
        ui->label_11->setPixmap(pixmap);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void HelloGui::updateVideoFeeds(QLabel *label)
{
    if (videoCapture2.isOpened())
    {
        cv::Mat frame;
        videoCapture2 >> frame; // Capture a frame

        if (!frame.empty())
        {
            // Convert OpenCV Mat to Qt QImage
            QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
            qimg = qimg.rgbSwapped(); // OpenCV uses BGR, QImage uses RGB, so we need to swap channels

            // Resize the image to fit the label
            QPixmap pixmap = QPixmap::fromImage(qimg).scaled(label->size(), Qt::KeepAspectRatio);

            // Display the image in the QLabel
            label->setPixmap(pixmap);
        }
    }
}

void HelloGui::startVideoThreadss() {
    for (size_t i = 0; i < video_pathss.size(); ++i) {
        video_threadss.push_back(std::thread([this, i]() {
            cv::VideoCapture capture(video_pathss[i]);
            QLabel* label = labelss_[i];
            while (true) {
                try {
                    if (!capture.isOpened()) {
                        // Handle error or break the loop
                        break;
                    }
                    cv::Mat frame;

                    capture >> frame; // Capture a frame

                    if (!frame.empty()) {
                        // Convert OpenCV Mat to Qt QImage
                        QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
                        qimg = qimg.rgbSwapped(); // OpenCV uses BGR, QImage uses RGB, so we need to swap channels

                        // Resize the image to fit the label
                        QPixmap pixmap = QPixmap::fromImage(qimg).scaled(label->size(), Qt::KeepAspectRatio);

                        // Display the image in the QLabel
                        label->setPixmap(pixmap);

                        // Sleep to control the frame rate
                        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Update every ~30 milliseconds for 30fps
                    } else {
                        // Handle empty frame
                        // Optionally, you can break the loop or take other actions
                    }
                } catch (const std::exception& e) {
                    // Handle exception
                    // Log the error or take appropriate action
                    // You can also break the loop if needed
                    break;
                }
            }
        }));
    }
}


void HelloGui::updatePolygon() {
    QFile file("/code_drive/src/template_gui_package/nodes/geofence.txt"); // Adjust path accordingly
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    QTextStream in(&file);
    QString coordinatesJsArray = "var latLngs = [";
    bool isEmpty = true; // Flag to track if the file is empty
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (!line.isEmpty()) { // Check if line is not empty
            isEmpty = false;
            QStringList parts = line.split(",");
            if (parts.size() == 2) {
                coordinatesJsArray += "[" + parts[0].trimmed() + ", " + parts[1].trimmed() + "],";
            }
        }
    }
    coordinatesJsArray.chop(1); // Remove the last comma
    coordinatesJsArray += "];";

    file.close();

    // Common JavaScript code to clear existing polygon.
    QString removePolygonJs = R"(
        if (window.currentPolygon) {
            window.currentPolygon.remove();
            window.currentPolygon = null;
        }
    )";

    QString jsCode;
    if (!isEmpty) {
        // Only add new polygon if the file is not empty and contains valid coordinates
        jsCode = coordinatesJsArray + R"(
            window.currentPolygon = L.polygon(latLngs, {color: '#39FF14'}).addTo(map_3387e5a10ff0da1a0bd5b3b27c4260fc);
        )";
    } else {
        // Optional: Handle the case when the file is empty, such as displaying a message
        jsCode = R"(
            console.log("No coordinates available to draw a polygon.");
        )";
    }

    // Execute the JavaScript code to remove any existing polygon and possibly add a new one
    jsCode = removePolygonJs + jsCode;
    webEngineView_map->page()->runJavaScript(jsCode);
}




double HelloGui::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    // Convert latitude and longitude from degrees to radians
    lat1 *= M_PI / 180.0;
    lon1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;

    // Earth radius in kilometers
    const double R = 6371.0;

    // Calculate differences in latitude and longitude
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Haversine formula
    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat1) * cos(lat2) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double distance = R * c;

    return distance;
}


void HelloGui::gpsInfoCallback(const std_msgs::String::ConstPtr& msg)
{
    // Extract number of satellites from the message and set it in valueStr
    std::istringstream iss(msg->data);
    std::string token;

    // Find the position of the colon
    size_t colonPos = msg->data.find(':');
    if (colonPos != std::string::npos) {
        // Extract the substring after the colon
        std::string substr = msg->data.substr(colonPos + 1);

        // Convert the substring to an integer
        int numSatellites = std::stoi(substr);

        // Convert the integer to a string
        std::ostringstream oss;
        oss << numSatellites;
        std::string valueStr = oss.str();

        // Set the value in label_12
        ui->label_17->setText(QString::fromStdString(valueStr));
    }
}

void HelloGui::homePositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // Extract latitude, longitude, and altitude from the message
    homeLatitude = msg->point.y;
    homeLongitude = msg->point.x;
    double altitude = msg->point.z;

    // Print the received home position data on the console
    //ROS_INFO("Home Position - Latitude: %f, Longitude: %f, Altitude: %f", latitude, longitude, altitude);
}




void HelloGui::eulerCallback(const std_msgs::String::ConstPtr& msg) {
    std::istringstream iss(msg->data);
    std::string token;

    while (std::getline(iss, token, ',')) {
        size_t pos = token.find(":");

            std::string valueStr = token.substr(pos + 1);
            
            if (token.find("Pitch:") != std::string::npos) {
                pitchValue = std::stof(valueStr); 
                QString jsCode = QString("pitchValueChanged(%1);").arg(pitchValue);
    webEngineView_->page()->runJavaScript(jsCode);
            }
            if (token.find("Roll:") != std::string::npos) {
                rollValue = std::stof(valueStr); 
                QString jsCode = QString("rollValueChanged(%1);").arg(rollValue);
    webEngineView_->page()->runJavaScript(jsCode);
            }
            if (token.find("Yaw:") != std::string::npos) {
                yawValue = std::stof(valueStr);
                QString jsCode = QString("yawValueChanged(%1);").arg(yawValue);
    webEngineView_->page()->runJavaScript(jsCode);
                yawValueMap2 = yawValue;
            }
        }
    

}


void HelloGui::updateDateTimeDisplay()
{
    QDateTime now = QDateTime::currentDateTime();
    QString dateString = now.toString("    yyyy-MM-dd");
    QString timeString = now.toString("     HH:mm:ss");

    ui->DATE->setText(dateString); // Update the date
    ui->TIME->setText(timeString); // Update the time
}


void HelloGui::batteryCallback(const std_msgs::String::ConstPtr& msg)
{
    // Extract battery percentage from the message based on the voltage and update label_12
    std::istringstream iss(msg->data);
    std::string token;

    while (std::getline(iss, token, ',')) {
        size_t pos = token.find("Voltage: ");
        if (pos != std::string::npos) {
            std::string valueStr = token.substr(pos + 9); // Extract voltage value
            float voltage = std::stof(valueStr);

            // Calculate battery percentage for a 2S1P battery from voltage (6.0V to 8.4V)
            float batteryPercentage = (voltage - 5.4) / (8.7 - 5.4) * 100;
            batteryPercentage = std::max(0.0f, std::min(100.0f, batteryPercentage)); // Clamp between 0% and 100%

            // Update the label with battery percentage rounded to 2 decimal places
            ui->label_12->setText(QString::number(batteryPercentage, 'f', 2) + "%");
            break;
        }
    }
}


void HelloGui::flightmodeCallback(const std_msgs::String::ConstPtr& msg)
{
    // Extract flight mode from the message and set it in label_12
    std::istringstream iss(msg->data);
    std::string token;

    // Directly reading the full message, assuming it contains "Flight Mode - Stabilized" or similar
    std::getline(iss, token); // Read the entire message into token

    size_t pos = token.find("Flight Mode -");
    if (pos != std::string::npos) {
        // Correctly adjust the offset to skip "Flight Mode - " and the space, extracting just the mode
        std::string valueStr = token.substr(pos + 14); // Skip "Flight Mode -" including the space after the dash

        // Update the label with the extracted flight mode
        ui->label_13->setText(QString::fromStdString(valueStr));
    }
}

void HelloGui::PositionmodeCallback(const std_msgs::String::ConstPtr& msg)
{
   std::string data = msg->data;

    size_t latPos = data.find("Lat: ");
    size_t longPos = data.find("Long: ");
    size_t altPos = data.find("Alt: ");
    size_t mPos = data.find(" m");

    if (latPos != std::string::npos && longPos != std::string::npos && altPos != std::string::npos) {
        std::string latStr = data.substr(latPos + 5, longPos - latPos - 7); // Adjusted substring length
        std::string longStr = data.substr(longPos + 6, altPos - longPos - 7); // Adjusted substring length
        std::string altStr = data.substr(altPos + 5, mPos - altPos - 5);
        // Convert string to float, then round to 2 decimal points
        float latFloat = std::stof(latStr);
        float longFloat = std::stof(longStr);
        float altFloat = std::stof(altStr);
                
        double distance = calculateDistance(homeLatitude, homeLongitude, latFloat, longFloat);
        
        globalLatitude2 = latFloat;
        globalLongitude2 = longFloat;
        
        //ROS_INFO("Current Latitude: %f, Current Longitude: %f", latFloat, longFloat);
        // Log the distance
        //ROS_INFO("Distance between home and current position: %.2f km", distance);

        std::stringstream latSS, longSS, altSS, distance_to_gsc;
        latSS << std::fixed << std::setprecision(6) << latFloat;
        longSS << std::fixed << std::setprecision(6) << longFloat;
        altSS << std::fixed << std::setprecision(2) << altFloat;
        distance_to_gsc << std::fixed << std::setprecision(2) << distance;

        // Update the labels with rounded values
        ui->label_14->setText(QString::fromStdString(latSS.str()));
        ui->label_15->setText(QString::fromStdString(longSS.str()));
        ui->label_16->setText(QString::fromStdString(altSS.str() + " m"));
        ui->label_23->setText(QString::fromStdString(distance_to_gsc.str() + " km"));
       
       
    }
}


void HelloGui::speedCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string data = msg->data;

    // Extracting speed value using regex
    std::regex rgx("Speed: ([\\d.]+) m/s");
    std::smatch match;

    if (std::regex_search(data, match, rgx) && match.size() == 2) {
        // Convert extracted string to float
        float speed = std::stof(match[1]);

        // Round the speed to 2 decimal places
        float roundedSpeed = std::round(speed * 100) / 100; // rounding to 2 decimal places

        // Setting the speed to the label
        std::stringstream speedStream;
        speedStream << std::fixed << std::setprecision(2) << roundedSpeed;
        std::string speedStr = speedStream.str();

        ui->label_18->setText(QString::fromStdString(speedStr + " m/s"));
    }
}


void HelloGui::updateMapYawInHTML(float percentage)
{
    // Use percentage to dynamically update the battery value in HTML
    QString jsCode = QString("yawMapValueChanged(%1);").arg(percentage);
    webEngineView_map->page()->runJavaScript(jsCode);
}


void HelloGui::renderHTMLMap(const QString& filePath)
{
    // Load HTML file into the QWebEngineView
    webEngineView_map->load(QUrl::fromLocalFile(filePath));
}


void HelloGui::renderHTML(const QString& filePath)
{
    // Load HTML file into the QWebEngineView
    webEngineView_->load(QUrl::fromLocalFile(filePath));
    
    // Disable scroll bars
}
