#include <QApplication>
#include <QIcon>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QDebug>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QWidget>
#include <QStackedWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QUiLoader>
#include <QObject>
#include <QWebEngineView>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QtWebEngineWidgets/QWebEngineView>
#include <QString>
#include <sstream>
#include <regex>
#include "hello_gui.h"
#include "ui_hello_gui.h"
//#include "MainWindow.h"
//#include "ui_MainWindow.h"
#include <iostream>


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "o2i", ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    // Create instances of HelloGui and MainWindow
    HelloGui w; 
    w.show();  // Show the HelloGui window

    // Create a ROS node handle
    ros::NodeHandle nh;



    int result = app.exec();
    ros::shutdown();
    return result;
}

