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
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <iostream>


int HelloGui::myStaticInt1 = 1;
int HelloGui::myStaticInt2 = 1;
int HelloGui::myStaticInt3 = 1;
int HelloGui::myStaticInt4 = 1;

void createStackedWidget(QStackedWidget *stackedWidget, HelloGui &w, MainWindow &mainWindow) {

    stackedWidget->addWidget(&w);
    stackedWidget->addWidget(&mainWindow);
}

cv::VideoWriter videoWriterTopic1;
cv::VideoWriter videoWriterTopic2;
cv::VideoWriter videoWriterTopic3;
cv::VideoWriter videoWriterTopic4;

void HelloGui::saveFrameFromTopic1() {
    saveTopic1 = !saveTopic1; // Toggle saveTopic1 flag
    
    if (saveTopic1) {
        qDebug() << "Recording start from left camera";
        timerTopic1 = new QTimer(this);
        connect(timerTopic1, &QTimer::timeout, this, &HelloGui::saveFrameTopic1Repeatedly);
        timerTopic1->start(33); // Save frames every second (adjust as needed)
        
    } else {
        timerTopic1->stop();
        delete timerTopic1;
        
        if (videoWriterTopic1.isOpened()) {
            videoWriterTopic1.release(); // Release the video writer
            qDebug() << "Video saved for left camera";
        }
        
    }
}
// Slot to handle button click for Topic 2
void HelloGui::saveFrameFromTopic2() {
    saveTopic2 = !saveTopic2; // Toggle saveTopic2 flag
    
    if (saveTopic2) {
        qDebug() << "Recording start from right camera";
        timerTopic2 = new QTimer(this);
        connect(timerTopic2, &QTimer::timeout, this, &HelloGui::saveFrameTopic2Repeatedly);
        timerTopic2->start(33); // Save frames every second (adjust as needed)
        
    } else {
        timerTopic2->stop();
        delete timerTopic2;
        
        if (videoWriterTopic2.isOpened()) {
            videoWriterTopic2.release(); // Release the video writer
            qDebug() << "Video saved for right camera";
        }
        
    }
}

void HelloGui::saveFrameFromTopic3() {
    saveTopic3 = !saveTopic3; // Toggle saveTopic3 flag
    
    if (saveTopic3) {
        qDebug() << "Recording start from front camera";
        timerTopic3 = new QTimer(this);
        connect(timerTopic3, &QTimer::timeout, this, &HelloGui::saveFrameTopic3Repeatedly);
        timerTopic3->start(33); // Save frames every second (adjust as needed)
        
    } else {
        timerTopic3->stop();
        delete timerTopic3;
        
        if (videoWriterTopic3.isOpened()) {
            videoWriterTopic3.release(); // Release the video writer
            qDebug() << "Video saved for front camera";    
        }
        
    }
}

void HelloGui::saveFrameFromTopic4() {
    saveTopic4 = !saveTopic4; // Toggle saveTopic4 flag
    
    if (saveTopic4) {
        qDebug() << "Recording start from back camera";
        timerTopic4 = new QTimer(this);
        connect(timerTopic4, &QTimer::timeout, this, &HelloGui::saveFrameTopic4Repeatedly);
        timerTopic4->start(33); // Save frames every second (adjust as needed)
        
    } else {
        timerTopic4->stop();
        delete timerTopic4;
        
        if (videoWriterTopic4.isOpened()) {
            videoWriterTopic4.release(); // Release the video writer
            qDebug() << "Video saved for back camera";
        }
        
    }
}

// Slot to save frame for Topic 1 periodically
void HelloGui::saveFrameTopic1Repeatedly() {
    if (!lastFrameTopic1.empty()) {
        std::string filename = "/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/rec/frame_topic1_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString() + ".jpg";
        cv::imwrite(filename, lastFrameTopic1);
        
        
        if (!videoWriterTopic1.isOpened()) {
            // Create VideoWriter object if not already opened
            std::string videoFilename = "/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/rec/Left_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString() + ".avi";
            videoWriterTopic1.open(videoFilename, cv::VideoWriter::fourcc('H','2','6','4'), 30, lastFrameTopic1.size());
        }
        
        if (videoWriterTopic1.isOpened()) {
            // Write frame to video
            videoWriterTopic1.write(lastFrameTopic1);
        }
        std::remove(filename.c_str());
    }
}

void HelloGui::saveFrameTopic2Repeatedly() {
    if (!lastFrameTopic2.empty()) {
        std::string filename = "/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/rec/frame_topic2_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString() + ".jpg";
        cv::imwrite(filename, lastFrameTopic2);
        //qDebug() << "Saved frame from topic 2 to" << QString::fromStdString(filename);
        
        if (!videoWriterTopic2.isOpened()) {
            // Create VideoWriter object if not already opened
            std::string videoFilename = "/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/rec/front_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString() + ".avi";
            videoWriterTopic2.open(videoFilename, cv::VideoWriter::fourcc('H','2','6','4'), 30, lastFrameTopic2.size());
        }
        
        if (videoWriterTopic2.isOpened()) {
            // Write frame to video
            videoWriterTopic2.write(lastFrameTopic2);
        }
        std::remove(filename.c_str());
    }
}

void HelloGui::saveFrameTopic3Repeatedly() {
    if (!lastFrameTopic3.empty()) {
        std::string filename = "/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/rec/frame_topic3_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString() + ".jpg";
        cv::imwrite(filename, lastFrameTopic3);
        //qDebug() << "Saved frame from topic 3 to" << QString::fromStdString(filename);
        
        if (!videoWriterTopic3.isOpened()) {
            // Create VideoWriter object if not already opened
            std::string videoFilename = "/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/rec/back_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString() + ".avi";
            videoWriterTopic3.open(videoFilename, cv::VideoWriter::fourcc('H','2','6','4'), 30, lastFrameTopic3.size());
        }
        
        if (videoWriterTopic3.isOpened()) {
            // Write frame to video
            videoWriterTopic3.write(lastFrameTopic3);
            
        }
        std::remove(filename.c_str());
    }
}

void HelloGui::saveFrameTopic4Repeatedly() {
    if (!lastFrameTopic4.empty()) {
        std::string filename = "/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/rec/frame_topic4_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString() + ".jpg";
        cv::imwrite(filename, lastFrameTopic4);
        //qDebug() << "Saved frame from topic 4 to" << QString::fromStdString(filename);
        
        if (!videoWriterTopic4.isOpened()) {
            // Create VideoWriter object if not already opened
            std::string videoFilename = "/home/kamikaze/Desktop/catkin_ws/src/template_gui_package/rec/right_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString() + ".avi";
            videoWriterTopic4.open(videoFilename, cv::VideoWriter::fourcc('H','2','6','4'), 30, lastFrameTopic4.size());
        }
        
        if (videoWriterTopic4.isOpened()) {
            // Write frame to video
            videoWriterTopic4.write(lastFrameTopic4);
        }
        std::remove(filename.c_str());
    }
}

void HelloGui::toggleButtonColor1() {
    myStaticInt1++;

    QPushButton* btn1 = qobject_cast<QPushButton*>(sender());
    if (btn1) {

        if (myStaticInt1%2 == 0) {
            btn1->setStyleSheet("background-color: red;"); // Change 'red' to your desired color

        } else {
            btn1->setStyleSheet("background-color: #39FF14;");
        }

        saveFrameFromTopic1(); // Correct way to call another member function
    }
}

void HelloGui::toggleButtonColor2() {
    myStaticInt2++;

    QPushButton* btn2 = qobject_cast<QPushButton*>(sender());
    if (btn2) {

        if (myStaticInt2%2 == 0) {
            btn2->setStyleSheet("background-color: red;"); // Change 'red' to your desired color

        } else {
            btn2->setStyleSheet("background-color: #39FF14;");
        }

        saveFrameFromTopic2(); // Correct way to call another member function
    }
}

void HelloGui::toggleButtonColor3() {
    myStaticInt3++;

    QPushButton* btn3 = qobject_cast<QPushButton*>(sender());
    if (btn3) {

        if (myStaticInt3%2 == 0) {
            btn3->setStyleSheet("background-color: red;"); // Change 'red' to your desired color

        } else {
            btn3->setStyleSheet("background-color: #39FF14;");
        }

        saveFrameFromTopic3(); // Correct way to call another member function
    }
}

void HelloGui::toggleButtonColor4() {
	
    myStaticInt4++;

    QPushButton* btn4 = qobject_cast<QPushButton*>(sender());
    if (btn4) {

        if (myStaticInt4%2 == 0) {
            btn4->setStyleSheet("background-color: red;"); // Change 'red' to your desired color

        } else {
            btn4->setStyleSheet("background-color: #39FF14;");
        }

        saveFrameFromTopic4(); // Correct way to call another member function
    }
}

QLabel *label_main;
QDialog *dialog_main;
QScrollArea *scroll_area;
QPushButton *armButton;

// Callback function to handle incoming messages on the /drone/statustext topic
void statusTextCallback(const std_msgs::String::ConstPtr& msg) {
    // Convert ROS message to QString
    QString statusMessage = QString::fromStdString(msg->data);
    

    // Get current time
    QString currentTime = QTime::currentTime().toString("HH:mm:ss");

    // Format message with time and set colors
    // Time in red and message in #39FF14, and set font size to 15
    QString formattedMessage = QString("<font color='red' size='4'>%1</font> <font color='#39FF14' size='4'>- %2</font>")
                               .arg(currentTime)
                               .arg(statusMessage);

    // Ensure we are in the GUI thread and append the message
}

// Callback function to handle incoming messages on the /drone/arm_ready topic
void armReadyCallback(const std_msgs::String::ConstPtr& msg) {
    QString status = QString::fromStdString(msg->data);
    
    

    // Update armButton text directly
    if (armButton) { // Ensure armButton is not nullptr
    	if (status == "Not ready to arm"){
    	armButton->setStyleSheet("font-family: 'Orbitron', sans-serif; color: red; font-size: 14pt; text-align: center; border: 1px solid red; font-weight: bold;");
    	    armButton->setText(status);
    	}else{
    	armButton->setStyleSheet("font-family: 'Orbitron', sans-serif; color: #39FF14; font-size: 14pt; text-align: center; border: 1px solid #39FF14; font-weight: bold;");
            armButton->setText(status); // Set the text of the button to the status message
            }
    } else {
        ROS_ERROR("armButton is nullptr");
    }
}



int main(int argc, char *argv[]) {
    ros::init(argc, argv, "o2i", ros::init_options::AnonymousName);
    QApplication app(argc, argv);

    // Create instances of HelloGui and MainWindow
    HelloGui w; 
    MainWindow mainWindow;

    // Create a stacked widget to switch between the two windows
    QStackedWidget *stackedWidget = new QStackedWidget();
    stackedWidget->addWidget(&w);
    stackedWidget->addWidget(&mainWindow);
    
    // Create a ROS node handle
    ros::NodeHandle nh;

    // Subscribe to the /drone/statustext topic
    ros::Subscriber statusSub = nh.subscribe("/drone/statustext", 10, statusTextCallback);

    // Subscribe to the /drone/arm_ready topic
    ros::Subscriber armReadySub = nh.subscribe<std_msgs::String>("drone_status", 1000, armReadyCallback);

    // Find and connect existing push buttons in HelloGui and MainWindow
    QPushButton *button1 = w.findChild<QPushButton*>("pushButton");
    QPushButton *button2 = mainWindow.findChild<QPushButton*>("pushButton_2");
    
    if (button1) {
        QObject::connect(button1, &QPushButton::clicked, [stackedWidget]() { stackedWidget->setCurrentIndex(1); });
    } else {
        qDebug() << "Failed to find existing push button in HelloGui";
    }

    if (button2) {
        QObject::connect(button2, &QPushButton::clicked, [stackedWidget]() { stackedWidget->setCurrentIndex(0); });
    } else {
        qDebug() << "Failed to find existing push button in MainWindow";
    }
	
    // Create and configure common buttons
    QPushButton *commonButton = new QPushButton("L");
    commonButton->setStyleSheet("background-color: #39FF14;");

    QPushButton *commonButton1 = new QPushButton("R");
    commonButton1->setStyleSheet("background-color: #39FF14;");

    QPushButton *commonButton2 = new QPushButton("F");
    commonButton2->setStyleSheet("background-color: #39FF14;");

    QPushButton *commonButton3 = new QPushButton("B");
    commonButton3->setStyleSheet("background-color: #39FF14;");

    // Create armButton
    armButton = new QPushButton();
    armButton->setStyleSheet("font-family: 'Orbitron', sans-serif; color: #39FF14; font-size: 14pt; text-align: center; border: 1px solid #39FF14; font-weight: bold;");

    // Connect button clicks to toggleButtonColor functions
    QObject::connect(commonButton, &QPushButton::clicked, &w, &HelloGui::toggleButtonColor1);
    QObject::connect(commonButton1, &QPushButton::clicked, &w, &HelloGui::toggleButtonColor2);
    QObject::connect(commonButton2, &QPushButton::clicked, &w, &HelloGui::toggleButtonColor3);
    QObject::connect(commonButton3, &QPushButton::clicked, &w, &HelloGui::toggleButtonColor4);
    
    // Set up the dialog for status messages
    dialog_main = new QDialog();
    dialog_main->setWindowTitle("Status Messages");
    dialog_main->setFixedSize(100, 100);
    dialog_main->setStyleSheet("background-color: black;");

    QVBoxLayout *mainLayout2 = new QVBoxLayout(dialog_main);

    scroll_area = new QScrollArea(dialog_main);
    scroll_area->setWidgetResizable(true);

    label_main = new QLabel();
    label_main->setWordWrap(true);
    scroll_area->setWidget(label_main);

    QPushButton *clearButton = new QPushButton("Clear");
    clearButton->setStyleSheet("background-color: #39FF14;");
    clearButton->setFixedSize(100, 25);

    QHBoxLayout *buttonLayout = new QHBoxLayout();
    buttonLayout->addStretch(1);
    buttonLayout->addWidget(clearButton);

    scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
    scroll_area->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    mainLayout2->addWidget(scroll_area);
    mainLayout2->addLayout(buttonLayout);

    QObject::connect(clearButton, &QPushButton::clicked, [label_main, scroll_area]() {
        label_main->clear();
        scroll_area->verticalScrollBar()->setValue(0);
    });

    QWidget *mainWidget = new QWidget;
    mainWidget->setGeometry(0, 0, 896, 414); // Set the window size
    QGridLayout *mainLayout = new QGridLayout();
    mainWidget->setLayout(mainLayout);

    mainLayout->addWidget(stackedWidget, 0, 0, 1, -1);
    
   // Connect the armButton's clicked signal to show the dialog_main
   QObject::connect(armButton, &QPushButton::clicked, [dialog_main]() {
        dialog_main->show(); // Show the dialog when armButton is clicked
    });

    // Assuming commonButton, commonButton1, commonButton2, and commonButton3 are initialized somewhere above as pointers to QPushButton
    // Directly setting geometry like this is unusual in layouts but here's how you'd correct the setParent usage:
    commonButton->setParent(mainWidget);
    commonButton1->setParent(mainWidget);
    commonButton2->setParent(mainWidget);
    commonButton3->setParent(mainWidget);
    armButton->setParent(mainWidget);

     // Set geometries after setting parent, if you really need to set geometries manually (not recommended when using layouts)
     commonButton->setGeometry(670, 10, 21, 21); // x, y, width, height
     commonButton1->setGeometry(690, 10, 21, 21);
     commonButton2->setGeometry(690, 30, 21, 30);
     commonButton3->setGeometry(670, 30, 21, 30);
     armButton->setGeometry(60, 20, 90, 25);

    mainWidget->setLayout(mainLayout);
    mainWidget->setWindowTitle("IRUS-O2I GCS");
    mainWidget->setStyleSheet("background-color: black;");
    mainWidget->show();

    int result = app.exec();
    ros::shutdown();
    return result;
}

