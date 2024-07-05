#include "MainWindow.h"
#include "ui_MainWindow.h"
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

using namespace std;

double pitchValues = 0.0;
double rollValues = 0.0;
double yawValues = 0.0;

float globalLatitude = 0.0;
float globalLongitude = 0.0;
float yawValueMap = 0.0;

cv::VideoCapture videoCapture;


MainWindow::MainWindow(QWidget *parent) 
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    nh_ = ros::NodeHandle("~");
    ros_timer = new QTimer(this);

    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    ros_timer->start(100);

    battery_sub_ = nh_.subscribe("/drone/battery", 1, &MainWindow::batteryCallback, this);
    speed_sub_ = nh_.subscribe("/drone/velocity", 1, &MainWindow::speedCallback, this);
    euler_sub_ = nh_.subscribe("/drone/euler", 1, &MainWindow::eulerCallback, this);
    flight_mode_sub_ = nh_.subscribe("/drone/flight_mode", 1, &MainWindow::flightmodeCallback, this);
    position_sub_ = nh_.subscribe("/drone/position", 1, &MainWindow::PositionmodeCallback, this);
    home_position_sub_ = nh_.subscribe("/drone/home_position", 1, &MainWindow::homePositionCallback, this);
    gps_info_sub_ = nh_.subscribe("/drone/gps_info", 1, &MainWindow::gpsInfoCallback, this);

    QTimer *dateTimeTimer = new QTimer(this);
    connect(dateTimeTimer, &QTimer::timeout, this, &MainWindow::updateDateTimeDisplay);
    dateTimeTimer->start(1000); // Update every second

    // Create a web engine view and set it up
    webEngineView_ = new QWebEngineView(this);
    QVBoxLayout *layout_map = new QVBoxLayout(ui->webEngineViewPlaceholder);
    layout_map->addWidget(webEngineView_); 
    
    webEngineView_map = new QWebEngineView(this);
    QVBoxLayout *layout = new QVBoxLayout(ui->webEngineViewPlaceholder_2);
    layout->addWidget(webEngineView_map);
    
    // Populate video_paths and labels_ vectors
    video_paths = {"/home/kamikaze/Downloads/beach_-_102359 (720p).mp4", "/home/kamikaze/Downloads/beach_-_102359 (720p).mp4", "/home/kamikaze/Downloads/beach_-_102359 (720p).mp4", "/home/kamikaze/Downloads/beach_-_102359 (720p).mp4"};
    labels_ = {ui->label, ui->label_2, ui->label_3, ui->label_5};

    // Start video feed threads
    startVideoThreads();

    
    // Render the HTML file to the webEngineViewPlaceholder
    
    renderHTML("/code_drive/src/template_gui_package/map_hud.html");


    connect(this, &MainWindow::pitchValueChanged, this, &MainWindow::updatePitchInHTML);
    connect(this, &MainWindow::rollValueChanged, this, &MainWindow::updateRollInHTML);
    connect(this, &MainWindow::yawValueChanged, this, &MainWindow::updateYawInHTML);
connect(ui->pushButton_4, &QPushButton::clicked, this, &MainWindow::on_pushButton_4_clicked);
    
    
    QIcon icon("/home/kamikaze/Downloads/3d-video-record-icon-png.webp"); // Specify the correct path to your image
    ui->pushButton_3->setStyleSheet("QPushButton { text-align: left; padding-right: 50px; }"
                                 "QPushButton:pressed { padding-right: 48px; }"); // Adj
    ui->pushButton_3->setIcon(icon);
    
    
    
    renderHTMLMap("/code_drive/src/template_gui_package/mainwindow_map.html");
    
    
    QTimer *fileUpdateTimer = new QTimer(this);
connect(fileUpdateTimer, &QTimer::timeout, this, &MainWindow::updatePolygon);
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
        QObject::connect(timer, &QTimer::timeout, [this, globalLatitude, globalLongitude]() mutable {
            if (globalLatitude != 0.0 && globalLongitude != 0.0){
		    QString jsCode = QString::asprintf("addAndRemoveMarker(%f, %f, %f);", globalLatitude, globalLongitude, yawValueMap);
		    webEngineView_map->page()->runJavaScript(jsCode);
            }
            
            //cout << "addAndRemoveMarker(" << globalLatitude << ", " << globalLongitude << ")" << endl;


        });

        // Start the timer to add and remove a marker every 0.5 seconds
        timer->start(5);
    }
});
     
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::spinOnce()
{
    if (!ros::ok()) {
        QApplication::quit();
    }
    ros::spinOnce();
}

void MainWindow::updatePolygon() {
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


void MainWindow::on_pushButton_4_clicked()
{
    bool ok;
    int numRows = QInputDialog::getInt(this, tr("Enter Number of Rows"),
                                       tr("Number of Lat-Long Pairs:"), 1, 1, 100, 1, &ok);
    if (!ok) return;

    QDialog dialog(this);
    dialog.setWindowTitle("Enter Lat-Long Pairs");
    QGridLayout layout(&dialog);

    QTableWidget table(numRows, 2);
    table.setHorizontalHeaderLabels(QStringList() << "Latitude" << "Longitude");
    layout.addWidget(&table, 0, 0, 1, 2);

    QPushButton okButton("OK", &dialog);
    QPushButton cancelButton("Cancel", &dialog);
    layout.addWidget(&okButton, 1, 0);
    layout.addWidget(&cancelButton, 1, 1);
    
    table.setStyleSheet(
        "QTableWidget {"
        "   background-color: grey;"        // Set background color
        "}"
        "QHeaderView::section {"
        "   background-color: grey;"        // Header background color
        "   color: red;"                     // Header text color
        "}"
        "QTableCornerButton::section {"
        "   background-color: grey;"        // Corner button background color
        "}"
        // Style for the QLineEdit when editing
        "QLineEdit {"
        "   color: black;"                   // Set text color to black when editing
        "   background-color: grey;"        // Set background to black when editing
        "}"
        // Style for the QLineEdit after editing
        "QLineEdit[readOnly=\"true\"] {"
        "   color: #39FF14;"                   // Set text color to #39FF14 after editing
        "   background-color: grey;"         // Keep background grey after editing
        "}"
    );


    connect(&okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
    connect(&cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

    if (dialog.exec() == QDialog::Accepted)
    {
        QStringList latLongList;
        for (int i = 0; i < numRows; ++i)
        {
            QTableWidgetItem* latItem = table.item(i, 0);
            QTableWidgetItem* longItem = table.item(i, 1);
            if (latItem && longItem)
            {
                latLongList << latItem->text() + "," + longItem->text();
            }
        }

        // Specify the file path where to save
        QString filePath = "/code_drive/src/template_gui_package/nodes/geofence.txt";
        QFile file(filePath);
        if (file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QTextStream out(&file);
            foreach (const QString &line, latLongList)
            {
                out << line << "\n";
            }
            file.close();
            // Optionally, inform the user that the save was successful
            QMessageBox::information(this, tr("Save Successful"), tr("The geofence data has been saved successfully."));
        }
        else
        {
            // If the file could not be opened, inform the user.
            QMessageBox::critical(this, tr("Error"), tr("Could not save the geofence data. Please check the file path and permissions."));
        }
    }
}
void MainWindow::gpsInfoCallback(const std_msgs::String::ConstPtr& msg)
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

void MainWindow::homePositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // Extract latitude, longitude, and altitude from the message
    homeLatitude = msg->point.y;
    homeLongitude = msg->point.x;
    double altitude = msg->point.z;

    // Print the received home position data on the console
    //ROS_INFO("Home Position - Latitude: %f, Longitude: %f, Altitude: %f", latitude, longitude, altitude);
}


void MainWindow::PositionmodeCallback(const std_msgs::String::ConstPtr& msg)
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
        
        globalLatitude = latFloat;
        globalLongitude = longFloat;
        
        double distance = calculateDistance(homeLatitude, homeLongitude, latFloat, longFloat);
        
        // Log the individual coordinates
	//ROS_INFO("Home Latitude: %f, Home Longitude: %f", homeLatitude, homeLongitude);
	//ROS_INFO("Current Latitude: %f, Current Longitude: %f", latFloat, longFloat);
	// Log the distance
        //ROS_INFO("Distance between home and current position: %.2f km", distance);

        std::stringstream latSS, longSS, altSS, distance_to_gsc;
        latSS << std::fixed << std::setprecision(5) << latFloat;
        longSS << std::fixed << std::setprecision(5) << longFloat;
        altSS << std::fixed << std::setprecision(2) << altFloat;
        distance_to_gsc << std::fixed << std::setprecision(2) << distance;

        // Update the labels with rounded values
        ui->label_4->setText(QString::fromStdString(latSS.str()));
        ui->label_16->setText(QString::fromStdString(longSS.str()));
        ui->label_14->setText(QString::fromStdString(altSS.str() + " m"));
        ui->label_23->setText(QString::fromStdString(distance_to_gsc.str() + " km"));
        
        
       
    }
}

double MainWindow::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
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

void MainWindow::flightmodeCallback(const std_msgs::String::ConstPtr& msg)
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

void MainWindow::eulerCallback(const std_msgs::String::ConstPtr& msg) {
    std::istringstream iss(msg->data);
    std::string token;

    while (std::getline(iss, token, ',')) {
        size_t pos = token.find(":");
        
        
            std::string valueStr = token.substr(pos + 1);
            
            if (token.find("Pitch:") != std::string::npos) {
                pitchValues = std::stof(valueStr); 
                emit pitchValueChanged(pitchValues);
            }
            if (token.find("Roll:") != std::string::npos) {
                rollValues = std::stof(valueStr); 
                emit rollValueChanged(rollValues);
            }
            if (token.find("Yaw:") != std::string::npos) {
                yawValues = std::stof(valueStr);
                emit yawValueChanged(yawValues);
                yawValueMap = yawValues;
            }
        
    }
}

void MainWindow::updateVideoFeed(QLabel *label)
{
    if (videoCapture.isOpened())
    {
        cv::Mat frame;
        videoCapture >> frame; // Capture a frame

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

void MainWindow::startVideoThreads() {
    for (size_t i = 0; i < video_paths.size(); ++i) {
        video_threads.push_back(std::thread([this, i]() {
            cv::VideoCapture capture(video_paths[i]);
            QLabel* label = labels_[i];
            while (true) {
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
                    std::this_thread::sleep_for(std::chrono::milliseconds(33)); // Update every ~30 milliseconds for 30fps
                }
            }
        }));
    }
}



void MainWindow::updateDateTimeDisplay()
{
    QDateTime now = QDateTime::currentDateTime();
    QString dateString = now.toString("    yyyy-MM-dd");
    QString timeString = now.toString("     HH:mm:ss");

    ui->DATE->setText(dateString); // Update the date
    ui->TIME->setText(timeString); // Update the time
}

void MainWindow::batteryCallback(const std_msgs::String::ConstPtr& msg)
{
    // Extract battery percentage from the message and set it in label_12
    std::istringstream iss(msg->data);
    std::string token;

    while (std::getline(iss, token, ',')) {
        size_t pos = token.find("Voltage: ");

        if (pos != std::string::npos) {
            std::string valueStr = token.substr(pos + 9);
            float valueFloat = std::stof(valueStr);
            float batteryPercentage = ((valueFloat - 18.0) / (25.2 - 18.0)) * 100;
	    if (batteryPercentage < 0){
		    // Update the label with battery percentage
	    	ui->label_12->setText(QString::number(0.0, 'f', 2) + "%");
            }
            else{
            	ui->label_12->setText(QString::number(batteryPercentage, 'f', 2) + "%");
            }
            break;
        }
    }
}

void MainWindow::speedCallback(const std_msgs::String::ConstPtr& msg)
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

        ui->label_15->setText(QString::fromStdString(speedStr + " m/s"));
    }
}


void MainWindow::updatePitchInHTML(float percentage)
{
    // Use percentage to dynamically update the battery value in HTML
    QString jsCode = QString("pitchValueChanged(%1);").arg(percentage);
    webEngineView_->page()->runJavaScript(jsCode);
}

void MainWindow::updateRollInHTML(float percentage)
{
    // Use percentage to dynamically update the battery value in HTML
    QString jsCode = QString("rollValueChanged(%1);").arg(percentage);
    webEngineView_->page()->runJavaScript(jsCode);
}

void MainWindow::updateYawInHTML(float percentage)
{
    // Use percentage to dynamically update the battery value in HTML
    QString jsCode = QString("yawValueChanged(%1);").arg(percentage);
    webEngineView_->page()->runJavaScript(jsCode);
}

void MainWindow::renderHTML(const QString& filePath)
{
    // Load HTML file into the QWebEngineView
    webEngineView_->load(QUrl::fromLocalFile(filePath));
    
    // Disable scroll bars
    QWebEnginePage *page = webEngineView_->page();
    //if (page) {
       // page->settings()->setAttribute(QWebEngineSettings::ShowScrollBars, false);
   // }
}

void MainWindow::updateMapYawInHTML(float percentage)
{
    // Use percentage to dynamically update the battery value in HTML
    QString jsCode = QString("yawMapValueChanged(%1);").arg(percentage);
    webEngineView_map->page()->runJavaScript(jsCode);
}

void MainWindow::renderHTMLMap(const QString& filePath)
{
    // Load HTML file into the QWebEngineView
    webEngineView_map->load(QUrl::fromLocalFile(filePath));
}

