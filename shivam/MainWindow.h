#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtWebEngineWidgets/QWebEngineView>
#include <QtWidgets>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <QTimer>
#include <QWidget>
#include <QLabel>
#include <QtWebEngineWidgets/QWebEngineView>
#include <geometry_msgs/PointStamped.h>

#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QScreen>
#include <QString>
#include <QProcess>
#include <QDir>
#include <vector>
#include <string>
#include <thread>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    

    void batteryCallback(const std_msgs::String::ConstPtr& msg);
    void speedCallback(const std_msgs::String::ConstPtr& msg);
    void eulerCallback(const std_msgs::String::ConstPtr& msg);
    void flightmodeCallback(const std_msgs::String::ConstPtr& msg);
    void PositionmodeCallback(const std_msgs::String::ConstPtr& msg);
    void gpsInfoCallback(const std_msgs::String::ConstPtr& msg);
    void updateDateTimeDisplay();
    void homePositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    double calculateDistance(double lat1, double lon1, double lat2, double lon2);
    void statusTextCallback(const std_msgs::String::ConstPtr& msg);
    void updatePolygon();
    
    
    

signals:
    void pitchValueChanged(float percentage);
    void rollValueChanged(float percentage);
    void yawValueChanged(float percentage);
    

private slots:
    void spinOnce();
    void updatePitchInHTML(float percentage);
    void updateRollInHTML(float percentage);
    void updateYawInHTML(float percentage);
    void updateMapYawInHTML(float percentage);

    
public slots:
    void updateVideoFeed(QLabel*);
    void startVideoThreads(); 
    void on_pushButton_4_clicked(); 
     
    
    
private:
    Ui::MainWindow *ui;
    ros::NodeHandle nh_;
    ros::Subscriber battery_sub_;
    ros::Subscriber speed_sub_;
    ros::Subscriber euler_sub_;
    ros::Subscriber flight_mode_sub_;
    ros::Subscriber position_sub_;
    ros::Subscriber home_position_sub_;
    ros::Subscriber gps_info_sub_;
    
    
    QTimer *ros_timer;
    QTimer *videoTimer;
    
    QWebEngineView *webEngineView_;
    QWebEngineView *webEngineView_map; 
    std::vector<ros::Subscriber> image_subs_;
    QString latestStatusMessage;

    void renderHTML(const QString& filePath);
    void renderHTMLMap(const QString& filePath);
    
    std::vector<std::string> video_paths;
    std::vector<QLabel*> labels_;
    std::vector<std::thread> video_threads; // Vector to hold video feed threads
    
    double homeLatitude;
    double homeLongitude;
    double currentLatitude;
    double currentLongitude;
    
    ros::Subscriber arm_ready_sub_;

    
};

#endif // MAINWINDOW_H

