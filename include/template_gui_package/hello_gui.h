#ifndef HELLOGUI_H
#define HELLOGUI_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <QTimer>
#include <QWidget>
#include <QLabel>
#include <QtWebEngineWidgets/QWebEngineView>
#include <geometry_msgs/PointStamped.h>
#include <QPushButton>
#include <QTimer>
#include <QScreen>
#include <QString>
#include <sensor_msgs/Image.h> // Include the sensor_msgs/Image header
#include <opencv2/core/mat.hpp> // Ensure this is included for cv::Mat
#include <QProcess>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <thread>

namespace Ui {
class HelloGui;
}

class HelloGui : public QWidget {
    Q_OBJECT

public:
    explicit HelloGui(QWidget *parent = nullptr);
    ~HelloGui();

    void batteryCallback(const std_msgs::String::ConstPtr& msg);
    void flightmodeCallback(const std_msgs::String::ConstPtr& msg);
    void PositionmodeCallback(const std_msgs::String::ConstPtr& msg);
    void speedCallback(const std_msgs::String::ConstPtr& msg);
    void eulerCallback(const std_msgs::String::ConstPtr& msg);
    void gpsInfoCallback(const std_msgs::String::ConstPtr& msg);
    void updateDateTimeDisplay();
    void updateHtmlContent();
    void updateMapYawInHTML(float percentage);
    void homePositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    double calculateDistance(double lat1, double lon1, double lat2, double lon2);
    void statusTextCallback(const std_msgs::String::ConstPtr& msg);
    void screenCaptureCallback(const sensor_msgs::ImageConstPtr& msg);
    void saveFrameTopic1Repeatedly(); 
    void saveFrameTopic2Repeatedly(); 
    void saveFrameTopic3Repeatedly(); 
    void saveFrameTopic4Repeatedly();
    void displayImage(const sensor_msgs::Image::ConstPtr& msg);
    
    void saveToFile();
    void clearFile();
    
    cv::VideoWriter videoWriterTopic1;
    cv::VideoWriter videoWriterTopic2;
    cv::VideoWriter videoWriterTopic3;
    cv::VideoWriter videoWriterTopic4;

    ros::Subscriber screen_capture_sub_;
    QString screen_capture_directory_;
    void saveFrameFromTopic1();
    void saveFrameFromTopic2();
    void saveFrameFromTopic3();
    void saveFrameFromTopic4();
    void updatePolygon();
    
    
    cv::Mat lastFrameTopic1, lastFrameTopic2, lastFrameTopic3, lastFrameTopic4;
    
    void toggleButtonColor1();
    void toggleButtonColor2();
    void toggleButtonColor3();
    void toggleButtonColor4();
    
    static int myStaticInt1;
    static int myStaticInt2;
    static int myStaticInt3;
    static int myStaticInt4;
  
    // Define boolean flags for each topic to control toggling
    bool saveTopic1 = false;
    bool saveTopic2 = false;
    bool saveTopic3 = false;
    bool saveTopic4 = false;

    // Define QTimer pointers for each topic to schedule saving
    QTimer *timerTopic1;
    QTimer *timerTopic2;
    QTimer *timerTopic3;
    QTimer *timerTopic4;
    
    QString latestStatusMessage;
    
signals:
    void yawMapValueChanged(float percentage);signals:
    
    
    
private slots:
    void spinOnce();

    
public slots:
    void updateVideoFeeds(QLabel*);
    void startVideoThreadss();

    

    
private:
    Ui::HelloGui *ui;
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> image_subs_;
    ros::Subscriber battery_sub_;
    ros::Subscriber flight_mode_sub_;
    ros::Subscriber position_sub_;
    ros::Subscriber speed_sub_;
    ros::Subscriber euler_sub_;
    ros::Subscriber home_position_sub_;
    ros::Subscriber gps_info_sub_;
    ros::Subscriber image_sub_;

    QTimer *ros_timer;
    
    
    QTimer *updateContentTimer; 


    QWebEngineView *webEngineView_map; 
    QWebEngineView *webEngineView_;
    void renderHTMLMap(const QString& filePath);
    void renderHTML(const QString& filePath);
    
    ros::Subscriber label_11_sub_; // Subscriber for label_11

    
    void createDialog(); 
    
    double homeLatitude;
    double homeLongitude;
    double currentLatitude;
    double currentLongitude;
    
    std::vector<std::string> video_pathss;
    std::vector<QLabel*> labelss_;
    std::vector<std::thread> video_threadss; // Vector to hold video feed threads
    
    
};

#endif // HELLOGUI_H

