#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include "std_msgs/String.h"
#include <geometry_msgs/PointStamped.h>
#include <memory>
#include <cmath>
#include <atomic>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>


using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;
using std::condition_variable;
using std::mutex;
using std::unique_lock;
using std::cout;
using std::endl;
using namespace std;



std::atomic<bool> ready_to_exit{false};
std::condition_variable abcd;
std::mutex mutex_;
mutex mtx;

std::atomic<bool> continue_running(true);
std::mutex velocity_mutex;
Offboard::VelocityBodyYawspeed current_velocity{0.0f, 0.0f, 0.0f, 0.0f};





struct Point {
    double lat;
    double lon;
};

std::vector<Point> geofence;
std::string geofenceFilePath = "/code_drive/src/template_gui_package/nodes/geofence.txt";

struct Coordinate {
    double latitude;
    double longitude;
};

struct Waypoint {
    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float speed_m_s;
};

std::vector<Waypoint> waypoints;
std::mutex waypoints_mutex;

// Function prototype

bool isInsideGeofence(const Point& point, const vector<Point>& geofence) {
    bool inside = false;
    for (size_t i = 0, j = geofence.size() - 1; i < geofence.size(); j = i++) {
        if (((geofence[i].lon > point.lon) != (geofence[j].lon > point.lon)) &&
            (point.lat < (geofence[j].lat - geofence[i].lat) * (point.lon - geofence[i].lon) / (geofence[j].lon - geofence[i].lon) + geofence[i].lat)) {
            inside = !inside;
        }
    }
    return inside;
}


// Shared structure for home position
struct HomePosition {
    double latitude_deg = std::numeric_limits<double>::quiet_NaN();
    double longitude_deg = std::numeric_limits<double>::quiet_NaN();
    mutex home_mutex;
} shared_home_position;


// Signal handling for graceful shutdown
void signal_handler(int signal) {
    if (signal == SIGINT) {
        cout << "SIGINT received, shutting down..." << endl;
        ready_to_exit = true;
        continue_running = false;
        abcd.notify_one();
        ros::shutdown();
    }
}

// Function to read geofence from a text file
std::vector<Point> readGeofenceFromFile(const std::string& filePath) {
    std::vector<Point> geofence;
    std::ifstream file(filePath);
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double lat, lon;
        char delimiter;

        if (iss >> lat >> delimiter >> lon) {
            geofence.push_back({lat, lon});
        }
    }

    return geofence;
}


std::string flight_mode_to_string(Telemetry::FlightMode flight_mode) {
    switch (flight_mode) {
        case Telemetry::FlightMode::Unknown: return "Unknown";
        case Telemetry::FlightMode::Ready: return "Ready";
        case Telemetry::FlightMode::Takeoff: return "Takeoff";
        case Telemetry::FlightMode::Hold: return "Hold";
        case Telemetry::FlightMode::Mission: return "Mission";
        case Telemetry::FlightMode::ReturnToLaunch: return "RTL";
        case Telemetry::FlightMode::Land: return "Land";
        case Telemetry::FlightMode::Offboard: return "Guided";
        case Telemetry::FlightMode::FollowMe: return "Follow Me";
        case Telemetry::FlightMode::Manual: return "Stabilize";
        case Telemetry::FlightMode::Altctl: return "Altitude Hold";
        case Telemetry::FlightMode::Posctl: return "Position Hold";
        case Telemetry::FlightMode::Acro: return "Acrobatic";
        case Telemetry::FlightMode::Stabilized: return "Stabilized";
        case Telemetry::FlightMode::Rattitude: return "Rattitude";
        default: return "Other";
    }
}

std::string publish_drone_state(const bool& is_armed, const Telemetry::Position& position, const double flyingAltitudeThreshold, bool all_ok) {
    std::string droneStatus;

    if (!is_armed) {
        if (all_ok) {
            droneStatus = "Ready to arm";
        } else {
            droneStatus = "Not ready to arm";
        }
    } else if (is_armed && position.relative_altitude_m > flyingAltitudeThreshold) {
        droneStatus = "Flying";
    } else {
        droneStatus = "Armed";
    }
    return droneStatus;
}


void process_statustext(const mavlink_message_t& message, ros::Publisher& statustext_pub) {
    mavlink_statustext_t statustext;
    mavlink_msg_statustext_decode(&message, &statustext);

    std::string severity;
    switch (statustext.severity) {
        case MAV_SEVERITY_EMERGENCY: severity = "emergency"; break;
        case MAV_SEVERITY_ALERT: severity = "alert"; break;
        case MAV_SEVERITY_CRITICAL: severity = "critical"; break;
        case MAV_SEVERITY_ERROR: severity = "error"; break;
        case MAV_SEVERITY_WARNING: severity = "warning"; break;
        case MAV_SEVERITY_NOTICE: severity = "notice"; break;
        case MAV_SEVERITY_INFO: severity = "info"; break;
        case MAV_SEVERITY_DEBUG: severity = "debug"; break;
        default: severity = "unknown"; break;
    }

    std::string severity_message = severity + ": " + statustext.text;
    std::cout << severity_message << std::endl;

    std_msgs::String msg;
    msg.data = severity_message;
    statustext_pub.publish(msg);
}

void updateGeofence(const ros::TimerEvent&) {
    geofence = readGeofenceFromFile(geofenceFilePath);
    // You might want to add some logging here to confirm the update
    //ROS_INFO("Geofence updated.");
}

void update_velocity_from_file(const std::string& file_name) {
    std::ifstream file;
    while (continue_running) {
        {
            std::lock_guard<std::mutex> lock(velocity_mutex);
            file.open(file_name);
            if (file >> current_velocity.forward_m_s >> current_velocity.right_m_s >> 
                     current_velocity.down_m_s >> current_velocity.yawspeed_deg_s) {
              //  std::cout << "Velocity updated: " 
              //            << current_velocity.forward_m_s << " " 
              //            << current_velocity.right_m_s << " " 
              //            << current_velocity.down_m_s << " " 
              //            << current_velocity.yawspeed_deg_s << "\n";
            } else {
                //std::cerr << "Failed to read velocity values from file\n";
                // Remove the return statement here
            }
            file.close();
        }
        sleep_for(milliseconds(100)); // Correctly use milliseconds for a 100ms sleep
    }
}

void updateVelocity(Offboard& offboard, const std::atomic<bool>& continue_running) {
    while (continue_running) {
        {
            std::lock_guard<std::mutex> lock(velocity_mutex);
            
            offboard.set_velocity_body(current_velocity);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // Command update frequency
    }
}

void monitorDroneTelemetry(ros::NodeHandle& nh) {
    std::signal(SIGINT, signal_handler);

    // Publishers for telemetry data
    ros::Publisher position_pub = nh.advertise<std_msgs::String>("drone/position", 100);
    ros::Publisher euler_pub = nh.advertise<std_msgs::String>("drone/euler", 10);
    ros::Publisher battery_pub = nh.advertise<std_msgs::String>("drone/battery", 10);
    ros::Publisher flight_mode_pub = nh.advertise<std_msgs::String>("drone/flight_mode", 10);
    ros::Publisher velocity_pub = nh.advertise<std_msgs::String>("drone/velocity", 10);
    ros::Publisher rssi_pub = nh.advertise<std_msgs::String>("drone/rssi", 10);
    ros::Publisher gps_info_pub = nh.advertise<std_msgs::String>("drone/gps_info", 10);
    ros::Publisher home_position_pub = nh.advertise<geometry_msgs::PointStamped>("drone/home_position", 10);
    ros::Publisher statustext_pub = nh.advertise<std_msgs::String>("drone/statustext", 10);
    
    
    
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14551");  //serial:///dev/ttyACM0:230400 //udp://:14551  //udp://0.0.0.0:14550 //udp://:14540

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return;
    }

    cout << "Waiting for system to connect..." << endl;

    auto prom = std::promise<std::shared_ptr<System>>();
    auto fut = prom.get_future();
    mavsdk.subscribe_on_new_system([&mavsdk, &prom, &fut] {
        auto system = mavsdk.systems().at(0);
        if (system->is_connected()) {
            if (fut.wait_for(seconds(0)) == std::future_status::timeout) {
                prom.set_value(system);
            }
        }
    });

    if (fut.wait_for(seconds(10)) == std::future_status::timeout) {
        std::cerr << "Connection timeout!" << std::endl;
        return;
    }

    auto system = fut.get();
    auto telemetry = Telemetry{system};
    auto mavlink_passthrough = MavlinkPassthrough{system};
    mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_STATUSTEXT, [&](const mavlink_message_t& message) {
        process_statustext(message, statustext_pub);
    });
    

    bool is_armed = false;
    Telemetry::Position current_position;
    bool all_ok = false;

    const double flyingAltitudeThreshold = 0.5;
    
    auto action = Action{system};
    
    ros::Timer geofence_timer = nh.createTimer(ros::Duration(0.5), updateGeofence);

    

    
    telemetry.subscribe_position([&action, &geofence](Telemetry::Position position) {
        Point current_location = {position.latitude_deg, position.longitude_deg};
        
        if (geofence.size() > 0){
        if (!isInsideGeofence(current_location, geofence)) {
            cout << "Geofence breached, initiating RTL." << endl;
        }
    }});


    // Define a ROS publisher
    ros::Publisher status_pub = nh.advertise<std_msgs::String>("drone_status", 100);
	
    // Subscribe to armed state
    telemetry.subscribe_armed([&](bool armed) {
        is_armed = armed;
        std::string status_msg = publish_drone_state(is_armed, current_position, flyingAltitudeThreshold, all_ok);
        std_msgs::String msg;
        msg.data = status_msg;
        status_pub.publish(msg);
    });

    // Subscribe to position
    telemetry.subscribe_position([&](Telemetry::Position position) {
        current_position = position;
        std::string status_msg = publish_drone_state(is_armed, current_position, flyingAltitudeThreshold, all_ok);
        std_msgs::String msg;
        msg.data = status_msg;
        status_pub.publish(msg);
    });

    // Subscribe to health status
    telemetry.subscribe_health_all_ok([&](bool health_ok) {
        all_ok = health_ok;
        std::string status_msg = publish_drone_state(is_armed, current_position, flyingAltitudeThreshold, all_ok);
        std_msgs::String msg;
        msg.data = status_msg;
        status_pub.publish(msg);
    });
    


    // Subscribe to telemetry data and publish to ROS topics
    telemetry.subscribe_position([&](Telemetry::Position position) {
    std::ostringstream pos_stream;
    pos_stream << std::fixed << std::setprecision(7); // Adjust precision as needed
    pos_stream << "Position - Lat: " << position.latitude_deg << 
               ", Long: " << position.longitude_deg <<
               ", Alt: " + std::to_string(position.relative_altitude_m) << " m";

    std_msgs::String msg;
    msg.data = pos_stream.str();
    position_pub.publish(msg);
});
    
    telemetry.subscribe_gps_info([&](Telemetry::GpsInfo gps_info) {
        std_msgs::String msg;
        msg.data = "GPS Info - Number of Satellites: " + std::to_string(gps_info.num_satellites);
        gps_info_pub.publish(msg);
    });

    telemetry.subscribe_attitude_euler([&](Telemetry::EulerAngle euler_angle) {
        std_msgs::String msg;
        msg.data = "Attitude - Yaw: " + std::to_string(euler_angle.yaw_deg) + "ø, Pitch: " +
                   std::to_string(euler_angle.pitch_deg) + "ø, Roll: " + 
                   std::to_string(euler_angle.roll_deg) + "ø";
        euler_pub.publish(msg);
    });

    telemetry.subscribe_battery([&](Telemetry::Battery battery) {
        std_msgs::String msg;
        msg.data = "Battery - " + std::to_string(battery.remaining_percent * 100) +
                   "%, Voltage: " + std::to_string(battery.voltage_v) + " V";
        battery_pub.publish(msg);
    });

    telemetry.subscribe_flight_mode([&](Telemetry::FlightMode flight_mode) {
        std_msgs::String msg;
        msg.data = "Flight Mode - " + flight_mode_to_string(flight_mode);
        flight_mode_pub.publish(msg);
    });

    telemetry.subscribe_velocity_ned([&](Telemetry::VelocityNed velocity) {
    double speed = sqrt(pow(velocity.north_m_s, 2) + pow(velocity.east_m_s, 2) + pow(velocity.down_m_s, 2));

    std_msgs::String msg;
    msg.data = "Speed: " + std::to_string(speed) + " m/s";
    velocity_pub.publish(msg);
	});

    telemetry.subscribe_home([&](Telemetry::Position home) {
    
    
    geometry_msgs::PointStamped home_position_msg;
    home_position_msg.header.stamp = ros::Time::now();
    home_position_msg.header.frame_id = "map"; // Assuming home position is in the map frame

    // Set home position coordinates
    home_position_msg.point.x = home.longitude_deg;
    home_position_msg.point.y = home.latitude_deg;
    home_position_msg.point.z = home.relative_altitude_m; // Assuming relative altitude is the z-coordinate

    // Publish home position
    home_position_pub.publish(home_position_msg);

    // Update shared home position
    {
        std::lock_guard<std::mutex> lock(shared_home_position.home_mutex);
        shared_home_position.latitude_deg = home.latitude_deg;
        shared_home_position.longitude_deg = home.longitude_deg;
    }
    });

    mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_RADIO_STATUS, [&](const mavlink_message_t& message) {
        if (message.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
            mavlink_radio_status_t radio_status;
            mavlink_msg_radio_status_decode(&message, &radio_status);
            std_msgs::String msg;
            msg.data = "RSSI: " + std::to_string(static_cast<int>(radio_status.rssi)) + " (raw value)";
            rssi_pub.publish(msg);
        }
    });
    
    Offboard offboard(system);
    
    // Set initial velocity
    offboard.set_velocity_body(current_velocity);
    std::thread velocityUpdateThread(updateVelocity, std::ref(offboard), std::ref(continue_running));
    
    velocityUpdateThread.join();

    cout << "Monitoring drone telemetry. Press Ctrl+C to exit..." << endl;

    // Wait efficiently until the program is signaled to exit
    std::unique_lock<std::mutex> lock(mtx);
    while (!ready_to_exit) {
        abcd.wait(lock);
    }
}


void homePositionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {

    double latitude = msg->point.y;
    double longitude = msg->point.x;
    //ROS_INFO("Lat: %f, Long: %f", msg->point.y, msg->point.x);
}

void statustextCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received statustext message: [%s]", msg->data.c_str());
}



int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "multi_video_publisher_threaded");
    //QApplication app(argc, argv); 
    ros::NodeHandle nh;



    // Start monitoring drone telemetry on a separate thread
    std::thread telemetry_thread(monitorDroneTelemetry, std::ref(nh));
    
    //ros::Subscriber statustext_sub = nh.subscribe("drone/statustext", 1000, statustextCallback);

    ros::Publisher home_position_pub = nh.advertise<geometry_msgs::PointStamped>("drone/home_position", 10);
    //ros::Subscriber arm_ready_sub = nh.subscribe("drone/arm_ready", 1000, armReadyCallback);
    


    // ROS Timer for periodic publishing of the home position
    ros::Timer home_position_timer = nh.createTimer(ros::Duration(1.0), [&](const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(shared_home_position.home_mutex);
        if (!std::isnan(shared_home_position.latitude_deg) && !std::isnan(shared_home_position.longitude_deg)) {
            geometry_msgs::PointStamped home_position_msg;
            home_position_msg.header.stamp = ros::Time::now();
            home_position_msg.header.frame_id = "map";
            home_position_msg.point.x = shared_home_position.longitude_deg;
            home_position_msg.point.y = shared_home_position.latitude_deg;
            home_position_pub.publish(home_position_msg);
        }
    });
    
    std::thread velocity_thread(update_velocity_from_file, "/code_drive/src/kamikaze_main/pj_tflite_track_deepsort/gui_data/txt_files/velocity.txt");
    velocity_thread.join();
    
    
    
    // Wait for the system to connect
    while (!system) {
        ros::spinOnce();
        this_thread::sleep_for(seconds(1));
    }

    //ros::Subscriber homePositionSub = nh.subscribe("drone/home_position", 1000, homePositionCallback);
    //std::thread screen_capture_thread(screenCaptureThreadFunction, std::ref(nh));

    // ROS loop that allows callbacks to be called
    ros::AsyncSpinner spinner(5); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();


    


    // Ensure all threads are properly joined before exiting
    if (telemetry_thread.joinable()) {
        telemetry_thread.join();
    }


    return 0;
}
